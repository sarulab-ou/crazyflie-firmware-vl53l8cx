# IMU（ジャイロ・加速度計）

対応ソース:
- ドライバ: `src/hal/src/sensors_bmi088_bmp3xx.c`（Crazyflie 2.1: Bosch BMI088 + BMP3xx）
- 予測ステップ: `src/modules/src/kalman_core/kalman_core.c` の `predictDt()`
- サブサンプラ: `src/modules/src/axis3fSubSampler.c`

IMU は EKF の「観測」ではなく**予測ステップの入力（制御入力 u 相当）**として使われる。
ジャイロが姿勢を、加速度計が速度・位置を前進させる。

## 1. センサードライバでの前処理（sensorsTask, 1kHz）

BMI088 の割り込み（1kHz）ごとに `sensorsTask`（`sensors_bmi088_bmp3xx.c:293-378`）が生データを処理する。

### ジャイロ（BMI088 gyro, ±2000 dps）

処理パイプライン（`sensors_bmi088_bmp3xx.c:326-335`）:

```
生値 (int16 LSB)
  → バイアス減算            (gyroRaw − gyroBias)
  → スケーリング            × (2×2000/65536) [deg/s per LSB]
  → 機体座標系への回転       sensorsAlignToAirframe()  （IMU 取付角 imuPhi/Theta/Psi の補正）
  → 2次バターワース LPF     カットオフ 80 Hz
  → estimatorEnqueue(MeasurementTypeGyroscope)   単位: deg/s
```

**バイアス推定**（`processGyroBias`, `sensors_bmi088_bmp3xx.c:749-768`）:
- 起動後、直近 512 サンプルのリングバッファで分散を監視し、
  分散が閾値（`GYRO_VARIANCE_BASE = 100`）を下回った＝**機体が静止している**と判断できたときの
  平均値をバイアスとして採用する。
- 軽量版（`GYRO_BIAS_LIGHT_WEIGHT`）では単純に最初の 1000 サンプルの平均を使う。
- バイアスが見つかるまで機体は「未校正」扱いで離陸できない。

### 加速度計（BMI088 accel, ±24 g）

処理パイプライン（`sensors_bmi088_bmp3xx.c:338-347`）:

```
生値 (int16 LSB)
  → スケーリング            × (2×24/65536) [g per LSB]
  → スケール補正            ÷ accScale
  → 機体座標系への回転       sensorsAlignToAirframe()
  → 重力方向への整列         sensorsAccAlignToGravity()  （設定ブロックの roll/pitch トリムで回転）
  → 2次バターワース LPF     カットオフ 30 Hz
  → estimatorEnqueue(MeasurementTypeAcceleration)   単位: g
```

**スケール補正**（`processAccScale`, `sensors_bmi088_bmp3xx.c:681-696`）:
- ジャイロバイアス確定後（＝静止確認後）、200 サンプル分の加速度ノルム |a| の平均を `accScale` とし、
  以後の測定を割る。静止時の重力が正確に 1.000 g になるようゲイン誤差を自己校正する仕組み。

## 2. EKF タスクでの受け取りとサブサンプリング

`updateQueuedMeasurements()`（`estimator_kalman.c:350-357`）で:

- `MeasurementTypeGyroscope` → `gyroSubSampler` に累積 ＋ `gyroLatest` として保持
  （`gyroLatest` は Flow deck の観測モデルにもそのまま渡される。[03-flow-deck.md](03-flow-deck.md) 参照）
- `MeasurementTypeAcceleration` → `accSubSampler` に累積 ＋ `accLatest` として保持
  （`accLatest` は外部化時に world 座標系加速度として出力される）

予測ステップ（100Hz）の直前に `axis3fSubSamplerFinalize()` が呼ばれ、
**蓄積した約10サンプルの平均**をとりつつ単位変換する:

- ジャイロ: deg/s × `DEG_TO_RAD` → **rad/s**
- 加速度: g × `GRAVITY_MAGNITUDE (9.81)` → **m/s²**

## 3. 予測ステップでの利用（predictDt, kalman_core.c:340-604）

連続時間のダイナミクス（Mueller の論文に基づく）:

```
ẋ = R (I + [[d]]) p          位置 ← 姿勢×機体速度
ṗ = f/m e3 − [[ω]] p − g (I − [[d]]) R⁻¹ e3    機体速度 ← 推力・コリオリ項・重力
ḋ = ω                        姿勢誤差 ← ジャイロ角速度
```

（[[·]] は外積行列、ω はジャイロ測定値、e3 = [0,0,1]ᵀ、f/m は質量正規化推力）

### ジャイロの役割 = 姿勢の伝播

- **クォータニオン積分**（`kalman_core.c:566-600`）:
  角速度 ω を dt 積分した微小回転 δq を作り、姿勢クォータニオンに乗算する。
  ```
  angle = |ω| dt,  δq = (cos(angle/2), sin(angle/2)・ω dt/angle)
  q ← δq ⊗ q  →  正規化
  ```
- **機体速度のコリオリ補正**: 速度状態は機体座標系なので、機体が回転すると
  速度ベクトルの見え方が変わる。これを `−[[ω]] p` 項（`gyro->z * tmpSPY − gyro->y * tmpSPZ` など）で補正。
- **姿勢誤差共分散の伝播**: 状態遷移行列 A の D0〜D2 ブロックに
  d = ω dt/2 を使った 2 次近似の回転補正が入る（Covariance Correction 論文の式）。
- 非飛行時は `attitudeReversion` により初期クォータニオンへ僅かに引き戻す（地上ドリフト対策）。

### 加速度計の役割 = 速度・位置の伝播

**飛行中**（`quadIsFlying == true`, `kalman_core.c:506-542`）:
- 使うのは **z 軸加速度（zacc = acc->z）のみ**。クアッドの推力は機体 z 方向にしか出ないため、
  飛行中の x, y 加速度計値は空力・振動ノイズとみなして使わない。
- 更新式（機体座標系）:
  ```
  pż += dt (zacc + ωy px − ωx py − g R22 − drag)
  pẋ += dt (      ωz py − ωy pz − g R20 − drag)
  pẏ += dt (      ωx pz − ωz px − g R21 − drag)
  ```
  重力は回転行列 R の第3行で機体座標系に射影して減算する。
- 抗力モデル（フラッピング機向け, `dragB_x/y/z`）と圧力中心オフセット（`cop_x/y/z`）が
  パラメータで有効化できる。
- 位置は機体座標系での変位 `(px dt, py dt, pz dt + zacc dt²/2)` を R でグローバル座標に回して積算。

**非飛行中**（自由落下・手持ちなど）:
- 加速度計 3 軸すべてを使い、`ṗ = a − [[ω]]p − g R⁻¹e3` で更新。

### プロセスノイズ

`addProcessNoiseDt()`（`kalman_core.c:613-641`）で毎ループ加算:
- 位置: `(procNoiseAcc·dt² + procNoiseVel·dt + procNoisePos)²`
- 速度: `(procNoiseAcc·dt + procNoiseVel)²`
- 姿勢: `(measNoiseGyro·dt + procNoiseAtt)²` — **ジャイロの測定ノイズがここで姿勢の不確かさとして注入される**
  （roll/pitch と yaw で別パラメータ `measNoiseGyro_rollpitch` / `measNoiseGyro_yaw`）。

## 4. 出力（externalize）

`kalmanCoreExternalizeState()`（`kalman_core.c:774-821`）:
- 姿勢: クォータニオン → オイラー角（yaw/pitch/roll, deg）。pitch は legacy 座標系向けに符号反転。
- 速度: 機体座標系状態を R でグローバル座標へ回転。
- 加速度: 最新の加速度計値（g 単位）を R でグローバル座標へ回転し、z から 1g を引いて重力を除去。
