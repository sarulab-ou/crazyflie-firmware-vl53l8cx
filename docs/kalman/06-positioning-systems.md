# その他の測位系（UWB / Lighthouse / 外部位置・ポーズ / ヨー誤差）

Flow deck + ToF 構成では使われないが、EKF が受け付ける残りの観測モデルをまとめる。
いずれも `estimator_kalman.c` の `updateQueuedMeasurements()` から呼ばれる。

## 1. UWB TWR — 既知点までの距離（mm_distance.c）

- 発生源: Loco Positioning deck の TWR（Two-Way Ranging）モード（`lpsTwrTag.c`）。
  アンカー（位置既知の UWB 基地局）との電波往復時間から**アンカーまでの距離 [m]** を得る。
- 測定データ: `distanceMeasurement_t { x, y, z（アンカー位置）, distance, stdDev }`
- 観測モデル（`kalmanCoreUpdateWithDistance`）:
  ```
  予測距離 d̂ = |p − p_anchor| = √(dx² + dy² + dz²)
  h[X] = dx/d̂,  h[Y] = dy/d̂,  h[Z] = dz/d̂   （視線方向の単位ベクトル）
  ```
  イノベーション `distance − d̂` でスカラー更新。
- robust 版 `kalmanCoreRobustUpdateWithDistance`（`mm_distance_robust.c`）:
  M 推定（Huber 系の重み付け）で外れ値の影響を抑える。パラメータ `kalman.robustTwr` で切替。
- 効果: 位置 (X, Y, Z) の絶対観測。複数アンカーからの距離で三辺測量的に位置が決まる。

## 2. UWB TDoA — 2 アンカー間の距離差（mm_tdoa.c）

- 発生源: Loco deck の TDoA2 / TDoA3 モード（`lpsTdoa2Tag.c` / `lpsTdoa3Tag.c`）。
  2 つのアンカーからの受信時刻差から**距離差 [m]** を得る（タグは受信のみ、多数機で共有可）。
- 測定データ: `tdoaMeasurement_t { anchorPositions[2], distanceDiff, stdDev }`
- 観測モデル（`kalmanCoreUpdateWithTdoa`）:
  ```
  d1 = |p − anchor1|,  d0 = |p − anchor0|
  予測値 = d1 − d0
  h[X] = dx1/d1 − dx0/d0   （Y, Z も同様）
  ```
- **外れ値フィルタが必須**: TDoA はマルチパスで大きな外れ値が出るため、
  `outlierFilterTdoaValidateIntegrator()`（イノベーションの大きさを積分器で監視し、
  異常が続くと測定を弾く）を通ったサンプルのみ更新に使う。
  robust 版（`mm_tdoa_robust.c`, M 推定）は `kalman.robustTdoa` で切替。
- 効果: 位置の絶対観測（双曲面上への拘束を多数重ねて位置が決まる）。

## 3. Lighthouse — スイープ角（mm_sweep_angles.c）

- 発生源: Lighthouse deck（`lighthouse.c`）。SteamVR ベースステーションが回転して掃く
  レーザー平面を機体上の受光センサーが検出した時刻から、
  **基地局から見たセンサーの角度（スイープ角）[rad]** を得る。
- 測定データ: `sweepAngleMeasurement_t { sensorPos（機体上のセンサー位置）,
  rotorPos / rotorRot / rotorRotInv（基地局の位置・姿勢）, measuredSweepAngle, t（回転面の傾き）,
  calibrationMeasurementModel（基地局の校正込み予測関数）, stdDev }`
- 観測モデル（`kalmanCoreUpdateWithSweepAngles`）:
  1. センサー位置を機体座標 → グローバル座標へ（EKF の R を使用）→ 機体位置に加算。
  2. 基地局ローカル座標系へ変換（rotorRotInv）。
  3. 校正モデルで予測スイープ角を計算し、実測角との差をイノベーションとする。
  4. ヤコビアンを基地局座標系で解析的に計算し、グローバル座標へ回転して h[X], h[Y], h[Z] に格納。
  5. Lighthouse 用外れ値フィルタ（`outlierFilterLighthouseValidateSweep`）を通過したもののみ更新。
- 効果: 位置の絶対観測（mm 級精度）。センサー位置のレバーアームに EKF の姿勢 R が使われるため、
  間接的に姿勢とも結びつく。

## 4. 外部位置（mm_position.c）

- 発生源: モーションキャプチャや外部測位を CRTP（`extpos`）で送り込む場合。
- 測定データ: `positionMeasurement_t { x, y, z, stdDev }`
- 観測モデル: `h[X+i] = 1` の直接観測を x, y, z の 3 回のスカラー更新として実行。
- 効果: 位置の完全な絶対観測。

## 5. 外部ポーズ（mm_pose.c）

- 発生源: 位置＋姿勢クォータニオンを送る外部システム（MoCap 等、CRTP `extpose`）。
- 測定データ: `poseMeasurement_t { pos, quat, stdDevPos, stdDevQuat }`
- 観測モデル:
  - 位置: mm_position と同じ直接観測 ×3。
  - **姿勢**: 測定クォータニオンと EKF クォータニオンの残差回転
    `q_res = q_meas ⊗ q_ekf⁻¹` を計算し、小角度近似
    `err = 2/q_res.w × Im(q_res)` で 3 軸の姿勢誤差ベクトルに変換。
    これを `h[D0]=1`, `h[D1]=1`, `h[D2]=1` の 3 回のスカラー更新として
    姿勢誤差状態 (D0, D1, D2) に直接入れる。
- 効果: **EKF で姿勢（ロール・ピッチ・ヨーすべて）を直接観測できる唯一の汎用経路**。
  ジャイロ積分のヨードリフトも完全に補正できる。

## 6. ヨー誤差（mm_yaw_error.c）

- 発生源: **Lighthouse deck のみ**（`lighthouse_position_est.c` の `estimateYaw()` /
  `estimateYawDeltaOneBaseStation()`）。`estimatorEnqueueYawError()` を呼ぶモジュールは
  リポジトリ全体でここ1箇所しかなく、AI deck 等の他のデッキからは呼ばれていない
  （AI deck ドライバ `aideck.c` はブートローダ／CPX 通信のみで、姿勢・位置推定には一切関与しない）。
- 仕組み（`estimateYawDeltaOneBaseStation`, `lighthouse_position_est.c:392-434`）:
  1. 1つの基地局から機体上の4つの受光センサーへのレイを計算し、
     機体デッキ平面との交点 `intersectionPoints[4]` を求める。
  2. EKF の現在の推定位置・姿勢（`R`）から、各センサーの実座標 `sensorPoints[4]` を計算する。
  3. 対角線ペア（センサー 0-3, 1-2）それぞれについて、交点側ベクトルとセンサー側ベクトルの
     向きのずれを `lighthouseGeometryYawDelta()` で角度差に変換し、2本の平均を `yawDelta` とする。
  4. `yawErrorMeasurement_t { yawError: yawDelta, stdDev: 0.01（固定値） }` を作って
     `estimatorEnqueueYawError()` でキューに投入
     （`CONFIG_DECK_LIGHTHOUSE_AS_GROUNDTRUTH` 有効時は Lighthouse 側を真値として使うため投入しない）。
- 測定データ: `yawErrorMeasurement_t { yawError, stdDev }`
- 観測モデル（`mm_yaw_error.c:28-35`）: `h[KC_STATE_D2] = 1`、
  イノベーション `S[KC_STATE_D2] − yawError` のスカラー更新。ヨー方向の姿勢誤差状態のみを直接補正する。
- 効果: 磁力計を持たない Crazyflie で、Lighthouse deck 使用時に限りヨー角のドリフトを
  絶対補正できる手段。Lighthouse を使わない構成（Flow deck + ToF のみ等）ではこの経路は
  一切使われず、ヨーはジャイロ積分のままドリフトし続ける。

## まとめ: どの状態を誰が観測するか

| 状態 | 予測（入力） | 観測（補正） |
|---|---|---|
| 位置 X, Y | IMU 積分 | UWB (TWR/TDoA), Lighthouse, 外部位置/ポーズ |
| 位置 Z | IMU 積分 | ToF（対地）, 気圧計（無効が既定）, 絶対高度, UWB, Lighthouse, 外部位置/ポーズ |
| 速度 PX, PY | 加速度計＋ジャイロ | **Flow deck**（唯一の速度直接観測） |
| 速度 PZ | 加速度計 | （直接観測なし。Z 位置観測との相関で補正） |
| 姿勢 D0, D1 (roll/pitch) | ジャイロ積分 | 外部ポーズのみ直接。それ以外は重力方向との整合（予測モデル内）と各観測の相関で間接補正 |
| 姿勢 D2 (yaw) | ジャイロ積分 | 外部ポーズ, ヨー誤差。無ければドリフトする |
