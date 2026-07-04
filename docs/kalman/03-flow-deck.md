# Flow deck（PMW3901 光学フローセンサー）

対応ソース:
- デッキドライバ: `src/deck/drivers/src/flowdeck_v1v2.c`
- センサードライバ: `src/drivers/src/pmw3901.c`（SPI motion burst 読み出し）
- 観測モデル: `src/modules/src/kalman_core/mm_flow.c`

Flow deck は PMW3901 光学フローセンサー（下向きカメラ）と ToF 距離センサー
（v1: VL53L0x, v2: VL53L1x）を搭載する。本ページはフロー側のみ。ToF 側は
[04-tof-height.md](04-tof-height.md) を参照。

## 1. センサーが出す生データ

PMW3901 は下向きの低解像度カメラ（35×35 px 相当）で、フレーム間の画像相関から
**累積ピクセル変位** を出力する。`pmw3901ReadMotion()` の motion burst で得られる
`motionBurst_t`:

| フィールド | 意味 |
|---|---|
| `deltaX`, `deltaY` | 前回読み出しからの累積ピクセル変位 (int16, 実測で 10× モーションピクセル単位) |
| `motion` | モーション検出フラグ（0xB0 で有効データ） |
| `squal` | Surface quality。追跡できた特徴量の数（0 = フロー取得不能） |
| `shutter` | シャッター時間（露光の長さ。テクスチャの少なさの指標） |
| `maxRawData` / `minRawData` / `rawDataSum` | 画像画素の統計 |

## 2. ドライバでの前処理（flowdeckTask）

`flowdeck_v1v2.c:87-188`。タスクは 100 tick 周期でポーリングし、以下を行う:

1. **軸の入れ替え・反転**（センサー取付方向の補正）:
   ```c
   accpx = -deltaY;   // 機体 X 方向のピクセル変位
   accpy = -deltaX;   // 機体 Y 方向のピクセル変位
   ```
2. **外れ値除去**: |変位| ≥ 100 px（`OULIER_LIMIT`）のサンプルは棄却し `outlierCount` を加算。
3. **測定ノイズ標準偏差の決定**:
   - 既定: 固定値 `flowStdFixed = 2.0`（パラメータで変更可）
   - `motion.adaptive` 有効時: シャッター時間から線形フィットで推定
     `std = 0.0007984 × shutter + 0.4335`（下限 0.1）。
     テクスチャが少ない路面ほど露光が延び、ノイズ大として扱われる。
4. **dt の計測**: 前回読み出しからの経過時間を µs タイマーで取得（測定は「dt の間の累積変位」なので必須）。
5. （オプションで移動平均 `USE_MA_SMOOTHING` / 1次 IIR `USE_LP_FILTER` による平滑化。既定は生値）
6. **キュー投入**: `motion == 0xB0`（有効フロー）かつ無効化パラメータが立っていない場合のみ
   `estimatorEnqueueFlow()` で `flowMeasurement_t { dpixelx, dpixely, stdDevX, stdDevY, dt }` を投入。

## 3. 観測モデル（kalmanCoreUpdateWithFlow, mm_flow.c:41-105）

### 何を「予測」するか

状態（機体速度・高度・回転行列）とジャイロから、
「この dt の間にカメラが観測するはずのピクセル変位」を予測し、実測ピクセル変位と比較する。

カメラ定数: `Npix = 35` px、`thetapix = 0.71674` rad（画角 42° に対応する地面長）。

まず、デッキが機体重心からずれて取り付けられている場合のレバーアーム補正
（`flowdeckPos` パラメータ、ω × r）を入れた**カメラ位置での実効機体速度**を作る:

```
v_cam_x = px + (ωy·rz − ωz·ry)
v_cam_y = py + (ωz·rx − ωx·rz)
```

予測ピクセル数（X 方向。Y も対称）:

```
predictedNX = (dt·Npix/θpix) · ( v_cam_x · R22 / z  −  ωy )
```

- 第1項: 並進による見かけの地面の流れ。高度 z で割る（近いほど速く流れる）。
  `R22`（回転行列の[2][2]成分 = 機体 z 軸とグローバル z 軸の内積）で傾き補正。
- 第2項: **機体の回転（ピッチ角速度）そのものによる画像の流れ**。並進とは無関係に
  カメラが振られると画像は流れるので、ジャイロ値（`gyroLatest`、deg/s→rad/s 変換）で差し引く。
- z は特異点回避のため 0.1 m 未満に飽和（`z_g = max(z, 0.1)`）。

実測値は `measuredNX = dpixelx × FLOW_RESOLUTION`（`FLOW_RESOLUTION = 0.10`。
センサーが 10× 単位で変位を返すため 1/10 に換算）。

### ヤコビアン H

X 方向更新（1×9、非ゼロ成分のみ）:

```
∂NX/∂Z  = (Npix·dt/θpix) · R22·v_cam_x · (−1/z²)   ← 高度にも感度あり
∂NX/∂PX = (Npix·dt/θpix) · R22/z                   ← 主に機体 X 速度を観測
```

Y 方向更新も同様に `KC_STATE_Z` と `KC_STATE_PY` に感度を持つ。
X, Y は **2 回の独立なスカラー更新**として処理され、観測ノイズは
`stdDev × FLOW_RESOLUTION` が使われる。

### 状態推定への効果

- 直接観測されるのは**機体座標系の水平速度 (PX, PY)**。GPS のない室内で速度ドリフトを抑える
  主要な情報源であり、ToF 高度と組み合わせて位置ホールドを可能にする。
- H が Z にも成分を持つため高度情報も僅かに入るが、主目的は速度。
- 位置 (X, Y) は直接観測されない（速度の積分なので長時間では位置ドリフトが残る）。
- 姿勢への直接の観測はないが、EKF の相関（共分散の非対角項）を通じて
  速度誤差の修正が姿勢誤差 (D0, D1) の修正にも波及する。
