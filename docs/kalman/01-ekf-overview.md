# EKF の全体構造

対応ソース:
- `src/modules/src/kalman_core/kalman_core.c`（フィルタ本体）
- `src/modules/src/estimator/estimator_kalman.c`（FreeRTOS タスク・キュー処理）

理論的なベースは以下の論文:
- Mueller, Hamer, D'Andrea, *"Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"* (ICRA 2015)
- Mueller, Hehn, D'Andrea, *"Covariance Correction Step for Kalman Filtering with an Attitude"* (JGCD 2016)

## 状態ベクトル

EKF が推定する誤差状態は 9 次元（`kalman_core.h` の `KC_STATE_*`）:

| 状態 | 記号 | 意味 | 座標系 | 単位 |
|---|---|---|---|---|
| `KC_STATE_X, Y, Z` | x, y, z | 位置 | グローバル（慣性）座標系 | m |
| `KC_STATE_PX, PY, PZ` | px, py, pz | 速度 | **機体（ボディ）座標系** | m/s |
| `KC_STATE_D0, D1, D2` | d0, d1, d2 | 姿勢誤差（ロール・ピッチ・ヨー方向） | 機体座標系 | rad（Rodrigues パラメータ） |

姿勢そのものはこの 9 次元状態には含まれず、**クォータニオン `q[4]` として別に保持**される
（`kalmanCoreData_t.q`）。D0〜D2 は「クォータニオンに対する小さな誤差」を表し、
毎回の finalize でクォータニオンに吸収されてゼロにリセットされる（誤差状態カルマンフィルタ / MEKF 方式）。
クォータニオンから回転行列 `R[3][3]` も導出・保持され、機体座標⇔グローバル座標の変換に使われる。

共分散行列は `P[9][9]`。上限 `MAX_COVARIANCE = 100`、下限 `MIN_COVARIANCE = 1e-6` でクランプされ、
対称性が毎回強制される。

## タスクの動作フロー（estimator_kalman.c）

`kalmanTask` は安定化ループ（1kHz）からセマフォで起床され、毎ループ以下を行う
（`estimator_kalman.c:216-287`）:

1. **予測ステップ（100Hz）** — `PREDICT_RATE = RATE_100_HZ`
   - IMU 測定（1kHz で届く）は `Axis3fSubSampler_t` に蓄積され、予測時に**平均化**して使う。
     このときジャイロは deg/s → rad/s（`DEG_TO_RAD`）、加速度は G → m/s²（`GRAVITY_MAGNITUDE = 9.81`）に変換される
     （`axis3fSubSampler.c:42-54`, `estimator_kalman.c:382-383`）。
   - `kalmanCorePredict()` で状態と共分散を前進（詳細は [02-imu.md](02-imu.md)）。
2. **プロセスノイズ加算** — `kalmanCoreAddProcessNoise()`（予測とは独立に毎ループ、経過時間 dt に応じて）。
3. **測定更新** — `updateQueuedMeasurements()`（`estimator_kalman.c:301-367`）。
   測定キュー（`estimatorDequeue`）に溜まった全測定を取り出し、型ごとに観測モデルへ振り分ける:

   | `MeasurementType` | 観測モデル | 主な発生源 |
   |---|---|---|
   | `Gyroscope` | （更新でなく予測用に蓄積） | IMU |
   | `Acceleration` | （同上） | IMU |
   | `Flow` | `kalmanCoreUpdateWithFlow` | Flow deck (PMW3901) |
   | `TOF` | `kalmanCoreUpdateWithTof` | Z-ranger / Flow deck の VL53L1x |
   | `Barometer` | `kalmanCoreUpdateWithBaro`（デフォルト無効） | BMP3xx |
   | `TDOA` | `kalmanCoreUpdateWithTdoa`（robust 版あり） | Loco deck (TDoA2/3) |
   | `Distance` | `kalmanCoreUpdateWithDistance`（robust 版あり） | Loco deck (TWR) |
   | `SweepAngle` | `kalmanCoreUpdateWithSweepAngles` | Lighthouse deck |
   | `Position` | `kalmanCoreUpdateWithPosition` | 外部位置（MoCap 等） |
   | `Pose` | `kalmanCoreUpdateWithPose` | 外部ポーズ（位置+姿勢） |
   | `AbsoluteHeight` | `kalmanCoreUpdateWithAbsoluteHeight` | 絶対高度源 |
   | `YawError` | `kalmanCoreUpdateWithYawError` | AI deck 等のヨー補正 |

4. **finalize** — `kalmanCoreFinalize()`（後述）。
5. **健全性チェック** — `kalmanSupervisorIsStateWithinBounds()`。状態が発散していたらフィルタをリセット。
6. **外部化** — `kalmanCoreExternalizeState()` で `state_t`（位置・速度・姿勢・加速度）を生成し、
   安定化ループ（コントローラ）へ渡す。

## スカラー更新（全観測モデル共通）

ほぼ全ての観測モデルは `kalmanCoreScalarUpdate()`（`kalman_core.c:215-285`）を使う。
1 次元の観測 z、観測ヤコビアン H（1×9）、観測ノイズ標準偏差 σ を受け取り、標準的な EKF 更新を行う:

```
S = H P Hᵀ + σ²          （イノベーション共分散）
K = P Hᵀ / S             （カルマンゲイン, 9×1）
x ← x + K・error          （error = 実測値 − 予測値）
P ← (KH−I) P (KH−I)ᵀ + K σ² Kᵀ   （Joseph 形式で数値的に安定な共分散更新）
```

多次元の観測（Flow の X/Y、位置の x/y/z など）は「スカラー更新の繰り返し」として実装される。

## finalize: 姿勢誤差のクォータニオンへの取り込み

`kalmanCoreFinalize()`（`kalman_core.c:651-772`）は測定更新後に呼ばれ:

1. 姿勢誤差 (d0, d1, d2) が十分大きければ（>0.1 mrad）、それを微小回転クォータニオン
   δq = (cos(θ/2), sin(θ/2)・v/θ)（θ=|v|）に変換し、現在の姿勢クォータニオンに乗算。
2. 「機体を回した」ことに対応して共分散行列も回転（Mueller の Covariance Correction、2次近似）。
3. クォータニオンから回転行列 R を再計算。
4. d0, d1, d2 を 0 にリセット。

これにより、次の予測ステップでは常に「姿勢誤差=0 の周りで線形化」できる。

## 飛行状態による切り替え

`supervisorIsFlying()` の結果（`quadIsFlying`）が予測モデルを切り替える:

- **飛行中**: 加速度計は機体 z 方向の推力のみを測ると仮定（x, y 加速度は無視。プロペラ推力は機体 z 軸方向にしか発生しないため）。抗力（drag）モデルも有効。
- **非飛行中**: 加速度は任意方向（自由落下・手持ち搬送など）として 3 軸すべて使用。
  さらに `attitudeReversion` パラメータにより、姿勢を初期クォータニオンへゆっくり引き戻す
  （地上での姿勢ドリフト防止。絶対姿勢が観測できるデッキ装着時は無効化される）。

## 座標系と単位の要点

- グローバル座標: X 前 / Y 左 / Z 上（ENU 系）。位置 [m]。
- 機体座標: X 前 / Y 左 / Z 上。速度状態 PX/PY/PZ はこの座標系 [m/s]。
- EKF 内部のジャイロは rad/s、加速度は m/s²。ただし**測定キューに入る時点では deg/s と G**であり、
  サブサンプラの変換係数で換算される点に注意。
