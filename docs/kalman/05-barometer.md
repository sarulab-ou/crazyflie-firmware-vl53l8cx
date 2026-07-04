# 気圧計（BMP3xx）

対応ソース:
- ドライバ: `src/hal/src/sensors_bmi088_bmp3xx.c`（`sensorsScaleBaro`, `sensorsTask` 内の気圧読み出し）
- 観測モデル: `src/modules/src/kalman_core/kalman_core.c` の `kalmanCoreUpdateWithBaro()`
  （`mm_absolute_height.c` と異なり、baro 用は kalman_core.c 内に直接実装されている）

## 1. センサーが出す生データ

BMP3xx（Crazyflie 2.1 では BMP388）は気圧 [Pa] と温度 [°C] を出力する。
IMU タスク（1kHz）内で 20 回に 1 回、すなわち **50 Hz**（`SENSORS_READ_BARO_HZ = 50`）で読み出される
（`sensors_bmi088_bmp3xx.c:350-368`）。

## 2. ドライバでの変換（sensorsScaleBaro, sensors_bmi088_bmp3xx.c:250-257）

気圧を国際標準大気の式で **ASL（海抜高度, Above Sea Level）[m]** に変換する:

```
pressure [hPa] = pressure [Pa] × 0.01
asl = ((1015.7 / p)^0.1902631 − 1) × (25 + 273.15) / 0.0065
```

（基準気圧 1015.7 hPa・基準気温 25 °C を仮定した簡易式。絶対値はあてにならないが
短時間の相対変化は数十 cm 精度で追える。）

`baro_t { pressure, temperature, asl }` を `MeasurementTypeBarometer` として
`estimatorEnqueue()` でキューに投入する。

## 3. 観測モデル（kalmanCoreUpdateWithBaro, kalman_core.c:324-338）

**注意: デフォルトでは無効。** `estimator_kalman.c:104` の `KALMAN_USE_BARO_UPDATE` が
コメントアウトされており、キューに入った気圧測定は捨てられる
（ToF や他の高度源がある構成では気圧計より高精度なため）。

有効化した場合の動作:

1. **基準高度の追跡**: 飛行していない間（`quadIsFlying == false`）は
   `baroReferenceHeight = 現在の ASL` を更新し続ける。
   → 離陸した瞬間の ASL が「地面 = Z 0 m」の基準になる。
2. 観測値: `meas = asl − baroReferenceHeight`（離陸地点からの相対高度 [m]）。
3. ヤコビアン: `h[KC_STATE_Z] = 1`（Z の直接観測）。
4. イノベーション `meas − Z` を、パラメータ `kalman.mNBaro`（`measNoiseBaro`）の
   標準偏差でスカラー更新。

### 状態推定への効果

- Z（高度）の絶対的なアンカー。ToF と違い床面の段差の影響を受けないが、
  ノイズが大きく（数十 cm オーダー）、気流・ドア開閉・天候でドリフトする。
- 姿勢・水平位置には直接寄与しない。

## 補足: mm_absolute_height（汎用の絶対高度観測）

`mm_absolute_height.c` の `kalmanCoreUpdateWithAbsoluteHeight()` は
気圧計とは別系統の汎用「絶対高度」観測モデルで、`heightMeasurement_t { height, stdDev }` を
そのまま Z の直接観測（h[Z] = 1）として更新する。
外部システムが `estimatorEnqueueAbsoluteHeight()` を呼ぶ場合に使われる
（基準高度の追跡のような処理は行わない）。
