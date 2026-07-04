# EKF (Kalman Filter) センサー統合ドキュメント

Crazyflie ファームウェアの EKF（Extended Kalman Filter）状態推定器において、
各センサー・デッキが「どのような生データから、どのようなデータを作り、
どのように状態推定（位置・速度・姿勢）に反映しているか」をまとめたドキュメント群。

## 目次

| ファイル | 内容 |
|---|---|
| [01-ekf-overview.md](01-ekf-overview.md) | EKF の全体構造。状態ベクトル、予測ステップ、測定キュー、finalize の流れ |
| [02-imu.md](02-imu.md) | IMU（BMI088 ジャイロ・加速度計）。バイアス推定、スケーリング、LPF、予測ステップでの利用 |
| [03-flow-deck.md](03-flow-deck.md) | Flow deck（PMW3901 光学フローセンサー）。ピクセル変位から機体速度の観測モデルまで |
| [04-tof-height.md](04-tof-height.md) | 下向き ToF（VL53L1x / Z-ranger、Flow deck 搭載分）と本リポジトリ独自の VL53L8CX デッキ |
| [05-barometer.md](05-barometer.md) | 気圧計（BMP3xx）。気圧→高度変換と（デフォルト無効の）高度更新 |
| [06-positioning-systems.md](06-positioning-systems.md) | その他の測位系: UWB（TWR / TDoA）、Lighthouse、外部位置・ポーズ、ヨー誤差 |

## 対応するソースコードの場所

- EKF 本体（予測・更新・finalize）: `src/modules/src/kalman_core/kalman_core.c`
- EKF タスク（キュー処理・スケジューリング）: `src/modules/src/estimator/estimator_kalman.c`
- 観測モデル（measurement model, `mm_*.c`）: `src/modules/src/kalman_core/`
- IMU ドライバ: `src/hal/src/sensors_bmi088_bmp3xx.c`
- デッキドライバ: `src/deck/drivers/src/`（`flowdeck_v1v2.c`, `zranger2.c`, `vl53l8cx_deck.c` など）

## データフローの全体像

```
[センサードライバ / デッキドライバ]
   生データ取得 → 校正・スケーリング・外れ値除去 → measurement_t を作成
        │  estimatorEnqueue()（測定キューへ投入）
        ▼
[estimator_kalman.c: kalmanTask]  (安定化ループから 1kHz でトリガ)
   ├─ 100Hz: 予測ステップ kalmanCorePredict()
   │     （ジャイロ・加速度計の平均値サブサンプルを入力）
   ├─ 毎ループ: プロセスノイズ加算
   ├─ 毎ループ: キューの全測定を消費し、種類ごとの観測モデル mm_* で更新
   │     （Flow, ToF, 気圧, TDoA, TWR, Lighthouse, 外部位置 ...）
   ├─ kalmanCoreFinalize(): 姿勢誤差状態をクォータニオンに取り込み
   └─ kalmanCoreExternalizeState(): state_t として制御系へ出力
```
