# 下向き ToF 距離センサー（高度観測）と VL53L8CX デッキ

対応ソース:
- Z-ranger v2 / Flow deck v2 の下向き ToF: `src/deck/drivers/src/zranger2.c`（VL53L1x）
- 旧 Z-ranger / Flow deck v1: `src/deck/drivers/src/zranger.c`（VL53L0x）
- 中継: `src/modules/src/range.c` の `rangeEnqueueDownRangeInEstimator()`
- 観測モデル: `src/modules/src/kalman_core/mm_tof.c`
- 本リポジトリ独自の VL53L8CX デッキ: `src/deck/drivers/src/vl53l8cx_deck.c`

## 1. センサーが出す生データ（VL53L1x）

VL53L1x はレーザー ToF（Time of Flight）方式の 1 点距離センサー。
`zRanger2Task`（`zranger2.c:116-146`）が 25 ms 周期（40 Hz）で距離 [mm] を読み出す。
設定は distance mode = MEDIUM、timing budget = 25 ms。

## 2. ドライバでの前処理

1. **外れ値除去**: 5000 mm 以上は棄却（`RANGE_OUTLIER_LIMIT`。センサーの実力上 >5 m は
   測れず、外れ値は典型的に 8 m 超として現れるため）。
2. **単位変換**: mm → m。
3. **距離依存の測定ノイズモデル**（`zranger2.c:47-52, 140-144`）:
   ```
   std(d) = 0.0025 × (1 + exp(k·(d − 2.5)))
   k = ln(0.2/0.0025) / (4.0 − 2.5)
   ```
   近距離（〜2.5 m）では std ≈ 2.5 mm、4 m で ≈ 0.2 m と指数的に悪化する経験モデル。
   遠くなるほど EKF での重みが自動的に下がる。
4. `rangeEnqueueDownRangeInEstimator()` → `tofMeasurement_t { distance, stdDev, timestamp }` を
   `estimatorEnqueueTOF()` でキューに投入。
   （同時に `rangeSet(rangeDown, ...)` でログ用の range モジュールにも書く。）

## 3. 観測モデル（kalmanCoreUpdateWithTof, mm_tof.c:28-61)

センサーは機体 z 軸方向（真下）の**斜距離**を測る。機体が傾くと床までの斜距離は
高度より長くなるため、傾き補正を入れて高度 Z の観測に変換する:

```
予測距離 h = z / cos(α)
α = |acos(R22)| − 7.5°   （負なら 0。R22 = 機体z軸と鉛直のなす角の cos）
```

- `R[2][2]` は回転行列の (2,2) 成分で、機体の傾き角 θ に対して cos θ に等しい。
- 7.5° の控除は ToF の測定コーン（FoV 約15°）の半分。傾いていても、コーン内の
  最近傍点（＝より鉛直に近い方向）を測ってしまう特性を近似的に補正している
  （出典: Lund 大学修士論文, mm_tof.c コメント参照）。
- **傾きすぎている場合は更新しない**: `R22 ≤ 0.1`（約 84° 以上傾斜）では発散を避けるため棄却。

ヤコビアン（非ゼロは 1 成分のみ）:

```
h[KC_STATE_Z] = 1 / cos(α)
```

イノベーション `(実測距離 − 予測距離)` とドライバ算出の stdDev でスカラー更新する。

### 状態推定への効果

- **高度 Z の絶対観測**。Flow deck とセットで使われることで、
  「フロー（速度）＋ ToF（高度）」により水平速度のスケールが正しく決まる
  （フローの観測式は v/z を含むため z の情報が不可欠）。
- 床からの相対距離なので、家具の上を通過するなどで床面高さが変わると
  Z 推定はその段差分ジャンプする（絶対高度ではなく対地高度の観測である点に注意）。

## 4. 本リポジトリの VL53L8CX デッキ（vl53l8cx_deck.c）

VL53L8CX はマルチゾーン ToF（4×4 = 16 ゾーンモードで使用、最大 8×8）。
本リポジトリでは **11 個の VL53L8CX センサー**（`vl53l8cx_NUM_SENSORS = 11`）を
SPI で多重化して扱う実験実装が入っている。

現状のデータフロー:

- 各センサー 16 ゾーンの距離 `distance_mm` と `target_status` を取得
  （読み出し本体は `system.c` 側の `Gget_Ranging()`）。
- グローバル配列に格納: `vl53l8cxToFDist[11][16]`（生距離 [mm]）、
  `vl53l8cxToFStatus[11][16]`（ゾーンごとの測定ステータス）、
  `vl53l8cxToFAvg[11]`（16 ゾーン平均 [mm]、ログ変数 `vl53l8cx.s0`〜`s10`）。

**重要: 現時点で VL53L8CX の測定値は EKF に投入されていない**
（`estimatorEnqueue` 系の呼び出しは無く、ログ・デバッグ出力のみ）。
EKF の姿勢・位置推定に使っているのはあくまで Flow deck 側の VL53L1x（下向き 1 点）である。

将来 EKF に統合する場合の素直な経路:

- 下向きセンサーの中央ゾーン（または status==5/9 の有効ゾーン平均）を
  `tofMeasurement_t` にして `estimatorEnqueueTOF()` に流す（既存の mm_tof がそのまま使える）。
- 壁向きセンサーを既知環境での距離観測として使うなら `distanceMeasurement_t`
  （mm_distance、既知点までの距離）や独自観測モデルの追加が必要。
