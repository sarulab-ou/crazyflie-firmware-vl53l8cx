# feat#7: Lighthouse を Ground Truth として microSD に記録する

## ⓪ 目的

VL53L8CX deck を用いた飛行システム（ToF オドメトリ／壁角度推定）の性能評価において、
比較対象となる **ground truth**（真値）を機体上で取得する。

- 評価対象（ToF 由来の推定）と真値（Lighthouse 由来の位置・姿勢）を同一機体・同一時刻で記録したい。
- ラジオ経由の log では帯域・取りこぼしの問題があるため、microSD card deck にオンボード記録する。
- 真値が推定器にフィードバックされると評価が循環参照になるため、Lighthouse → EKF の経路は切る。
- **機体上で記録するのは Lighthouse のスイープ角（層②）の生データのみとし、
  位置・姿勢の計算はオンボードでは一切行わない。すべて microSD の log 解析時にオフラインで行う。**
  解析は `~/workspace/crazyflie/crazyflie_rnd_lab/feat7_lighthouse/` で実施する（③）。

## ① 実装内容

### 1-1. 記録するのは層② のスイープ角のみ（オンボードでは位置を計算しない）

該当ファイル: `src/modules/src/lighthouse/lighthouse_core.c`

**方針: Lighthouse から得られるデータのうち、SD に記録するのは「層②＝スイープ角」だけとする。
位置・姿勢の計算は一切オンボードで行わず、すべて microSD の log 解析時にオフラインで行う。**

Lighthouse deck から取れるデータは 3 層あり、本 feat が使うのは層②:

| 層 | 内容 | 本 feat での扱い |
|---|---|---|
| ① UART フレーム | `pulseProcessorFrame_t`（sensor / timestamp / beamData / offset / channel）。12 B/パルス、4 BS で ~1600 フレーム/s | **使わない**（②③ 比較の結果、実装コスト・帯域ともに過大。②(i) 参照） |
| ② スイープ角 | `pulseProcessorResult_t angles` の `correctedAngles[]`（4 BS × 4 素子 × 2 角） | **これを記録する（主軸）** |
| ③ 派生値 | `lighthouse.x/y/z`（crossing beams）、yaw | **オンボードで計算しない。記録もしない** |

記録する層② の実体は `pulse_processor.h:230-245` の

```
angles.baseStationMeasurementsLh1[bs].sensorMeasurements[sensor].correctedAngles[0..1]
```

（LH2 の場合、`convertV2AnglesToV1Angles()` で V1 相当の方位角/仰角に変換済みのもの。`lighthouse_core.c:382`）。
4 BS × 4 素子 × 2 角 = **32 float = 128 B/サンプル**。30 Hz で約 4 kB/s。

**重要**: `pulseProcessorProcessed()` は `validCount` を 0 に戻すだけで角度 float はクリアしない
（`pulse_processor.c:146-154`）。したがって無効化された角度は log 上に前回値のまま残る。
新旧を区別する手段を必ずセットで記録しなければならない。

当初は `lighthouse.validAngles` を使う想定だったが、**LH2 では使えない**ことが分かった。
`pulseProcessorAnglesQuality()` が呼ぶ `pulseProcessorV2AnglesQuality()`
（`pulse_processor_v2.c:331-334`）は `// TODO krri Implement when LH2 is more mature` のまま
**常に 0 を返す**。そこで専用の log 変数を新規に追加した（1-2）。

ベースステーションの同定はオンボードで解決済みで、オフラインでやり直す必要はない:
LH2 ではデッキの FPGA が各パルスの `beamData`（LFSR）からチャンネル（0-15）を同定して
`pulseProcessorFrame_t.channel` に載せてくる（`lighthouse_core.c:266-268`）。
firmware 内は全て `[baseStation]` = チャンネル番号で添字付けされているため、
log 変数名がそのまま BS 番号に対応する。

### 1-2. ベースステーション数の記録

`lighthouse_core.c` に以下のビットマップが既にある（`lighthouse_core.c:102-117`）。

| 変数 | 意味 |
|---|---|
| `baseStationAvailabledMap` | システムに存在すると分かっている BS |
| `baseStationReceivedMap` | 直近に信号を受信した BS |
| `baseStationActiveMap` | 有効な角度データが得られた BS |
| `lighthouseCoreState.baseStationGeoValidMap` | ジオメトリが有効な BS（log: `lighthouse.bsGeoVal`） |

実装（`lighthouse_core.c`）:

| 追加した log 変数 | 型 | 意味 |
|---|---|---|
| `lighthouse.nBs` | uint8 | `baseStationReceivedMap` の popcount |
| `lighthouse.upd0`〜`upd3` | uint8 | **その BS の角度の更新カウンタ**（更新のたびに +1、255 で折り返す） |
| `lighthouse.mask0`〜`mask3` | uint8 | **直近の更新で完全なスイープ対を持っていた素子のビット field** |

`upd<bs>` と `mask<bs>` が stale 判定の本体。オフラインでは

- `upd<bs>` が前サンプルと同じ → その BS の角度は使い回し（無効）
- `mask<bs>` のビットが立っていない素子 → その素子の角度は古い（無効）

と判定する。`updateAngleLogInfo()` を `pulseProcessorProcessed()` が `validCount` を
潰す前に呼ぶことで実現している。ビットマップ（`bsReceive` / `bsActive`）もそのまま
log に含めるが、これらは 5 Hz 更新（`SYSTEM_STATUS_UPDATE_INTERVAL`）なので粗い補助情報。

**注意**: `baseStationActiveMap` は「ジオメトリが有効で、かつ EKF にデータを供給できる BS」を
意味する（`lighthouse_core.c:415-419` の `hasGeoData` 判定の中で立つ）。
本 feat では位置計算をオンボードで行わないので、**判定の主役は `baseStationReceivedMap`
（＝角度が受信できている BS）のほうになる**。ジオメトリはオフライン解析側で持つ（③ 3-2）。
`baseStationActiveMap` は「機上にジオメトリが入っているか」の確認用として補助的に記録する。

### 1-3. microSD card deck への書き込み

`src/deck/drivers/src/usddeck.c` は SD 上の `config.txt` に列挙した **log 変数名** を
固定周波数で書き出す仕組み（`on:fixedFrequency`）。ファーム側の変更は

- 素子 1-3 の角度 log を `LOG_ADD_DEBUG` から `LOG_ADD` に昇格
- BS 2/3 の角度 log を新規追加（従来は BS 0/1 の宣言しか無かった）
- 1-2 の `upd` / `mask` / `nBs` を追加

の 3 点。角度の log 変数名は **`lighthouse.angle<bs><x|y>_<sensor>`** に統一した
（BS 0-3 × 素子 0-3 × 2 スイープ = 32 個）。従来の `angle0x` / `angle0y` / `angle1x` /
`angle1y`（素子 0 のみ、サフィックス無し）は cfclient 等との互換のためエイリアスとして残してある。

SD に置く `config.txt` の実体は
**`~/workspace/crazyflie/crazyflie_rnd_lab/feat7_lighthouse/sdcard_config.txt`** に用意した。
中身は 46 変数で、内訳は

```
lighthouse.angle0x_0 .. angle3y_3   # 層②: 32 個
lighthouse.upd0 .. upd3             # 更新カウンタ
lighthouse.mask0 .. mask3           # 素子ビット field
lighthouse.nBs / bsReceive / bsActive
stabilizer.roll / pitch / yaw       # IMU 由来（yaw は参考）
```

に、評価対象側（`tofwall.*` など）を追加する形。

注意点:

- 30 Hz は `RATE_DO_EXECUTE(30, stabilizerStep)`（`stabilizer.c:367`）の整数除算で実効 30.3 Hz になる。
  時刻の基準は各レコード先頭に埋め込まれる `usecTimestamp()`（`usddeck.c:485,507`）を使うこと。
- `usddeckWriteEventData()` はリングバッファが満杯だと**黙ってサンプルを捨てる**（`usddeck.c:503`）。
  `usdLogStats.eventsRequested / eventsWritten` を確認し、取りこぼし率を見ること。
- 4 kB バッファでは層② の約 4 kB/s に対して 1 秒分しか猶予がない。SD の書き込みストール（FAT の
  クラスタ確保等で数百 ms）を吸収するため **buffer size は 8192 以上**にする。
- `config.txt` の 1 行は 32 文字までしか読まれない（`readBuffer[32]`, `usddeck.c:579`）。
  変数名は最長でも `lighthouse.angle0x_0` の 20 文字なので問題ないが、行末コメントを
  付けすぎないこと。

### 1-4. オンボードでの位置計算と EKF への注入を両方止める

要件は 2 段階ある。**(i) Lighthouse の計算結果を EKF に入れない**、
さらに **(ii) そもそも Lighthouse データから位置をオンボードで計算しない**。

#### (i) EKF への注入停止 — 既存 Kconfig で足りる

`CONFIG_DECK_LIGHTHOUSE_AS_GROUNDTRUTH`（`src/deck/drivers/src/Kconfig:261`）を有効にすると:

- `estimationMethod = 0`（crossing beams）に固定（`lighthouse_core.c:334-338`）
- `estimatorEnqueuePosition()` / `estimatorEnqueueSweepAngles()` / `estimatorEnqueueYawError()` を
  すべて `#ifndef` で無効化（`lighthouse_position_est.c:276, 307, 322, 354, 367, 457`）

参考: 無効化される EKF 更新の内訳（何が切れるのか）

| 経路 | 条件 | 更新する状態 |
|---|---|---|
| `kalmanCoreUpdateWithSweepAngles` | `estimationMethod == 1`（既定） | x, y, z のみ（`mm_sweep_angles.c:80-84`） |
| `kalmanCoreUpdateWithPosition` | `estimationMethod == 0` | x, y, z のみ（`mm_position.c:32-34`） |
| `kalmanCoreUpdateWithYawError` | **両方式で毎回** | yaw のみ（`KC_STATE_D2`, `mm_yaw_error.c:31`） |

**roll/pitch は元から Lighthouse では補正されていない**（IMU の重力基準のみ）。

#### (ii) 位置計算そのものの停止 — 追加で必要

(i) だけでは `estimatePositionCrossingBeams()` と `estimateYaw()` は**依然として毎周期実行される**
（結果を EKF に渡さないだけ）。本 feat では位置計算はオフラインで行うので、この計算自体が不要。

`usePulseResult()` の分岐（`lighthouse_core.c:424-433`）は

```c
switch(estimationMethod) {
  case 0: usePulseResultCrossingBeams(...); break;
  case 1: usePulseResultSweeps(...); break;
  default: break;          // ← 何もしない経路が既にある
}
```

となっており、**`estimationMethod` に 0/1 以外（例: 2）を入れれば位置計算は完全に走らなくなる**。
`lighthouse.method` は `PARAM_ADD_CORE`（`lighthouse_core.c:922`）で実行時変更可能。

実装（済）: `CONFIG_DECK_LIGHTHOUSE_AS_GROUNDTRUTH` 有効時の初期値を
`estimationMethod = 2`（＝「角度の復号のみ行い、位置推定はしない」）に変更した。
`lighthouse.method` は `PARAM_ADD_CORE` なので、実験中に 0（オンボード計算あり）へ
切り替えることもできる（③ 3-5 の移植検証で使う）。

これにより残る処理は「UART 復号 → 角度算出 → 校正適用 → V1 角への変換」までで、
`correctedAngles[]` は正しく更新され続ける（`convertV2AnglesToV1Angles()` は
`hasGeoData` チェックより前、`lighthouse_core.c:404-408` にあるため）。記録に必要な層② は保たれる。

**副作用と、その対処（済）**:

- `pulseProcessorClearOutdated()` と `pulseProcessorProcessed()` は `usePulseResultCrossingBeams()` /
  `usePulseResultSweeps()` の中でしか呼ばれない。位置計算を止めるとこれらが呼ばれなくなり、
  `validCount` がリセットされないので 1-2 の `mask` が飽和して stale 判定が壊れる。
  → `usePulseResultAnglesOnly()` を追加し、`estimationMethod == 2` のときに呼ぶようにした。
  呼び出し位置は `hasGeoData` の判定より**前**にしてある。オフラインで位置を出す構成では
  機上にジオメトリが無くても角度は記録できなければならないため。
- `throttleLh2Samples()`（最大 50 sample/s に間引き）は EKF 保護のためのものなので不要になる。
  間引きが掛からない分、角度は取れるだけ更新される（4 BS で ~200 更新/s）。30 Hz サンプリングでは
  どのみち間引かれるので実害はない。
- ジオメトリ（`bsGeometry`）はオンボードでは使われなくなるが、**オフライン計算に必須**なので
  実験前に必ず吸い出して `feat7_lighthouse/geometry/` に保存すること（③ 3-2）。
- 校正データ（OOTX）はオンボードで `correctedAngles` に適用済みなので、オフラインでは不要。

## ② 想定される問題点の議論

### (a) yaw の真値をどう作るか ★最大の論点

**(a-1) Lighthouse の生データのみで yaw を解けるか → 原理的には可能だが精度が足りない**

受光素子 4 個は機体座標で既知（`sensorDeckPositions`、15×30 mm の平面配置）、
各素子について BS からの方位ベクトルが 1 本得られるので、平面 P4P（ホモグラフィ）として
6-DoF 姿勢が解ける。BS 2 台なら観測は 2 倍で拘束としては過剰。

問題は**条件数**。yaw を δ 回転させたとき素子が動く距離は `r·δ`（r ≈ 17 mm）で、
距離 d ≈ 3 m の BS から見た角度変化は `r·δ/d` にしかならない。
LH2 の角度雑音を σ ≈ 0.5 mrad とすると検出限界の yaw は

```
δ ≈ σ·d/r = 0.0005 × 3 / 0.017 ≈ 0.09 rad ≈ 5°
```

4 素子 × 2 BS の平均で √8 ≈ 2.8 倍改善しても **1〜2° 程度のランダム誤差**が残る。
バイアスは無いので静止時の平均には使えるが、飛行中の逐次値としては
評価対象（ToF 壁角度推定）と同程度以下の分解能しかなく、真値として機能しない。
roll/pitch は面外回転がほぼ観測できず、さらに条件が悪いので実用外。

**(a-2) EKF の位置と姿勢を既知として yaw を解く案 → 方向性は正しいが位置依存が致命的**

未知数を yaw 1 個に絞れば条件数は劇的に改善し精度は出る。
「EKF に戻さずログだけ取る」という方針自体も正しい。
ただし `estimateYawDeltaOneBaseStation()` は `cfPos`（EKF 位置）でデッキ平面を定義して
レイを交差させる（`lighthouse_position_est.c:409`）。
本構成では EKF の位置は **ToF オドメトリ由来**になるため、
**評価対象の誤差がそのまま真値の yaw に流れ込み、評価が循環参照になる**。
ToF が 10 cm ずれれば yaw も歪む。よって EKF 位置には依存させない。

**(a-3) 参考：位置は Lighthouse、roll/pitch だけ IMU、yaw を解く（機上実装案 / 本 feat では不採用）**

```
位置        ← Lighthouse crossing beams（EKF 非依存、真値として独立）
roll/pitch  ← IMU（重力基準、ToF オドメトリの影響を受けない）
yaw         ← 上記を既知として Lighthouse 生角度から解く（ログのみ、EKF に戻さない）
```

要点は **roll/pitch は「EKF の状態」ではあるが実質は加速度計の重力方向で決まる絶対量**であること。
ToF オドメトリは位置・yaw にしか寄与しないので、roll/pitch 経由の汚染はほぼ無視できる。
一方 EKF の位置と yaw は ToF 由来なので使わない。この切り分けで真値の独立性が保てる。

実装は既存関数の引数の差し替えで済む:

- `cfPos` に `estimatorKalmanGetEstimatedPos()` ではなく **crossing beams の `positionLog[]`** を渡す
- `RR` は IMU の roll/pitch と「前フレームの Lighthouse yaw `lhYaw`」から組み立てる
- 戻り値を `lhYaw += yawDelta` と積む

`estimateYawDeltaOneBaseStation()` は「仮定した yaw に対する補正量」を返す構造
（`:424-429` で対角線ベクトルの角度差を取っている）なので、この積分は**不動点反復**になり、
数フレームで EKF yaw から完全に独立した値に収束する。初期値だけ EKF から取れば以降は自立する。

ただし本 feat ではこの機上実装は行わず、同じ考え方をオフラインで実行する（(a-4)）。
ここに書いた解法自体は ③ の `yaw.py` にそのまま移植する。

**(a-4) 本 feat の方針：位置も yaw も全部オフラインで解く** ★採用

(a-3) の機上実装は行わない。**位置・yaw ともにオンボードでは一切計算せず**、
層② のスイープ角と IMU の roll/pitch を SD に落とし、解析時に計算する。

理由:

- crossing beams（`lighthouse_position_est.c:228-283`）は
  「`correctedAngles` × 2 BS + `bsGeometry`」だけの純粋な幾何演算で、EKF も時系列も使わない。
  **角度とジオメトリが log にあればオフラインで完全に再現できる**。
- むしろオフラインのほうが条件を緩められる。機上は `sensorsUsed == PULSE_PROCESSOR_N_SENSORS`
  （4 素子全部）でないと結果を捨てるが、オフラインなら 2-3 素子でも解けるし、
  **4 BS のうち有効な全ペアを使った最小二乗**にもできる。
- yaw も同様に、機上で非線形最適化を書かずに済み、全素子・全 BS を使った最小二乗と
  時間方向の平滑化まで後から掛けられる。(a-1) の条件数の限界（1-2°）自体は
  情報量で決まるので後処理では破れないが、平滑化が使える分だけ有利。
- 機上の計算を止めることで CPU と SPI 帯域が空き、(c) の VL53L8CX との競合が緩む。

→ **オフライン側の実装は ③ に記述する。**

### (b) 角度データの欠損と stale サンプル
オンボードで位置を計算しなくなったので「位置が更新されない」問題は消えるが、
**同じ問題が角度そのものに残る**。機体が傾く／BS が遮蔽されると、その BS・その素子の
`correctedAngles[]` は更新されないまま前回値が残り続ける
（`pulseProcessorProcessed()` は `validCount` しかクリアしない。`pulse_processor.c:146-154`）。
30 Hz でスナップショットを取るだけの usddeck からは、新しい角度と古い角度の区別がつかない。

→ 1-2 で追加した `lighthouse.upd<bs>` / `lighthouse.mask<bs>` を必ず併記し、オフラインで
stale なサンプルを弾く（`parse_usdlog.freshness()`）。`lighthouse.validAngles` は
LH2 では常に 0 を返すので使えない（1-1 参照）。

なお、オフライン計算では firmware の「4 素子揃わないと捨てる」制約に縛られないので、
有効な素子・BS ペアだけで解くという回復手段が取れる（(a-4)）。

### (c) SPI バス競合（VL53L8CX deck と microSD deck）★実害が出やすい
- VL53L8CX は deck SPI を使い、CS に `DECK_GPIO_IO1 / TX2 / IO2` を使用（`platform_vl53l8cx.c:39-41`）。
- usddeck も同じ deck SPI、CS は `DECK_GPIO_IO4`（`usddeck.c:84`）。
- `spiBeginTransaction/EndTransaction` の mutex で排他は取れるが、
  VL53L8CX は 1 フレームで数百バイト〜数 KB を細切れの `spiExchange(1, ...)` で読むため
  **SD 書き込みが長時間ブロックされ、ログ落ち・ranging 周期の乱れが起きうる**。
- ボーレートも両者で異なる（SD は 2→21 MHz 切替）ため、`spiConfigureSlow/Fast` の
  設定漏れによる誤動作に注意。
- 対策: ログ周波数を 30 Hz に抑える（層② で ~4 kB/s）、buffer size を 8 kB 以上に増やす、
  オンボードの Lighthouse 位置計算を止めて CPU を空ける（1-4(ii)、実装済）、
  VL53L8CX の連続読み出しをバースト（`spiExchange(chunk_len, ...)`）に寄せる。

### (d) ピン競合の確認
- Lighthouse deck は UART1 使用（`lighthouse.c:92-93`, `usedGpio = 0`）→ SPI/IO4 とは衝突しない。
- ただし usddeck の別コンフィグ（`DECK_ALT`）では CS が `DECK_GPIO_RX2` になり、
  VL53L8CX の CS1 = `TX2` と同じ UART2 系ピン帯を使う。**通常版（IO4）構成を維持すること**。
- Kconfig 上 `DECK_LIGHTHOUSE` は `!DEBUG_PRINT_ON_UART1` を要求する点にも注意。

### (e) 3 枚デッキ同時搭載の物理的問題
Lighthouse + microSD + VL53L8CX（複数センサ）を同時搭載すると
重量・重心・電流が増え、飛行性能そのものが変わる。
**評価は必ず「この構成のまま」行い、デッキ有無で条件を変えない**こと。
また Lighthouse 受光部が VL53L8CX のセンサ・配線で遮蔽されないレイアウトが必要。

### (f) EKF を切ることによる飛行の成立性
`CONFIG_DECK_LIGHTHOUSE_AS_GROUNDTRUTH=y` にすると Lighthouse は EKF に一切寄与しない。
その状態で自律飛行させるには **別の位置ソース（ToF オドメトリ等）だけで飛ぶ**必要があり、
そもそも評価対象が未成熟なうちは機体が発散する。
→ 初期段階は手動飛行 or 治具で動かして真値と ToF 推定を突き合わせる。
なお EKF への注入自体は `#ifndef` のコンパイル時分岐のままだが、
`lighthouse.method`（`PARAM_ADD_CORE`）で 2 ⇔ 0 を実行時に切り替えられるので、
「オンボードで計算するかどうか」だけは飛行中でも変えられる。

### (g) 時刻同期
SD ログは stabilizer 同期（1 kHz ベース、30 Hz に分周）で書かれるが、
Lighthouse の角度更新は非同期（BS のスイープ周期依存。4 BS で ~200 更新/s）、
VL53L8CX は 5〜10 Hz。**各系統の更新カウンタ**（`vl53l8cxFrameSeq`, 1-2 の `lighthouse.upd<bs>`）を
一緒に記録しないと、オフラインで正しく時刻合わせできない。
共通の時刻基準は各レコード先頭に埋め込まれる `usecTimestamp()`（`usddeck.c:485,507`）を使う。

### (h) 座標系の不一致
Lighthouse はベースステーションのジオメトリで定義される世界座標系、
ToF オドメトリは初期姿勢基準の相対座標系。
比較には初期オフセット・回転の推定（軌跡アライメント）が必要で、
**評価指標は絶対位置誤差ではなく相対誤差（RPE）やアライメント後の ATE** にするのが妥当。

## ③ オフライン解析側の実装（`feat7_lighthouse/` への移植）

層② の生データ（スイープ角）の解析は
`~/workspace/crazyflie/crazyflie_rnd_lab/feat7_lighthouse/` で行う。
**オンボードから取り除いた位置・姿勢計算を、Python で解析できるようにこのフォルダへ移植した。**

### 3-1. 移植した関数

| firmware 側 | 関数 | 移植先 |
|---|---|---|
| `lighthouse_geometry.c:127` | `lighthouseGeometryGetRay()` | `geometry.get_ray()` |
| 同 `:116` | `lighthouseGeometryGetBaseStationPosition()` | `geometry.get_base_station_position()` |
| 同 `:61` | `intersect_lines()` | `geometry.intersect_lines()` |
| 同 `:103` | `lighthouseGeometryGetPositionFromRayIntersection()` | `geometry.get_position_from_ray_intersection()` |
| 同 `:142` | `lighthouseGeometryIntersectionPlaneVector()` | `geometry.intersection_plane_vector()` |
| 同 `:159` | `lighthouseGeometryGetSensorPosition()` | `geometry.get_sensor_position()` |
| 同 `:169` | `lighthouseGeometryYawDelta()` | `geometry.yaw_delta()` |
| `lighthouse_position_est.c:228` | `estimatePositionCrossingBeams()` | `crossing_beams.estimate_position_crossing_beams()` |
| 同 `:392` | `estimateYawDeltaOneBaseStation()` | `yaw.estimate_yaw_delta_one_bs()` |
| 同 `:212` | `sensorDeckPositions` | `constants.SENSOR_DECK_POSITIONS` |
| `kalman_core.c:799-808` | 四元数 → Euler（ZYX） | `constants.rotation_matrix()` / `attitude_from_log()` |

オフラインだから可能な拡張として、以下を追加した:

- `crossing_beams.estimate_position_multi_bs()` — 有効な BS を 3 台以上まとめて使い、
  素子ごとに全レイへの最小二乗最近接点を求める。4 素子揃わなくても解ける
  （ファームは `sensorsUsed == 4` でないと捨てる）
- `yaw.solve_yaw()` — `estimate_yaw_delta_one_bs()` を不動点反復して yaw を解く
- `yaw.solve_yaw_least_squares()` — 全 BS・全素子の角度残差を直接最小化する。
  `yaw.predict_angles()`（`get_ray()` の逆変換）を新規に起こしている

**移植しなかったもの**: 校正の適用（`lighthouseCalibrationApplyV1/V2()`）と
V2→V1 角の変換（`pulseProcessorV2ConvertToV1Angles()`）。`correctedAngles` を
記録しているのでどちらもオンボードで適用済み。パルス復号は層① を使わないので不要。

#### 符号について（要注意）

移植で 2 箇所、符号を合わせる必要があった。

1. **yaw の補正方向**: `yawDelta` は EKF に「yaw の誤差」として渡され、
   `mm_yaw_error.c:34` で `S[D2] - yawError` として**引かれる**。したがって
   不動点反復は `yaw -= delta` である（`yaw += delta` だとゲイン 2 で発散する）。
2. **log の pitch**: `stabilizer.pitch` は `kalman_core.c:807` の
   `.pitch = -pitch*RAD_TO_DEG` で符号が反転している。回転行列を組み直すときは
   戻すこと（`constants.attitude_from_log()`）。

### 3-2. 解析に必要な、log に含まれないデータ

**ベースステーションのジオメトリ（`baseStationGeometry_t`: 各 BS の `origin[3]` と `mat[3][3]`）は
SD の log には一切現れない。** これが無いと位置は計算できない。実験のたびに

```
python3 fetch_geometry.py --uri radio://0/80/2M/E7E7E7E7E7
```

で吸い出して `geometry/<日付>_bs_geometry.yaml` に保存する。cfclient の Lighthouse タブから
書き出した yaml（python タグ付き）もそのまま読める。

`sensorDeckPositions` は firmware 側の定数なので `constants.py` に写してある。

### 3-3. ディレクトリ構成

```
feat7_lighthouse/
├── README.md              # 使い方
├── sdcard_config.txt      # SD カードに置く config.txt の雛形（1-3）
├── constants.py           # sensorDeckPositions、回転行列の定義
├── geometry.py            # lighthouse_geometry.c の移植
├── crossing_beams.py      # 位置推定（ファーム互換版 + 拡張版）
├── yaw.py                 # yaw 解法（機上には実装しない）
├── parse_usdlog.py        # usddeck バイナリログのパースと stale 判定
├── analyze.py             # 通しの解析。CSV と図を出す
├── fetch_geometry.py      # BS ジオメトリの吸い出し
├── geometry/              # 吸い出したジオメトリ
├── logs/                  # SD から吸い出した生ログ
├── figs/                  # 出力図
└── tests/
    ├── test_port.py       # C 実装との数値比較 + 合成データでの復元確認
    ├── ref_geometry.c     # lighthouse_geometry.c をホストで動かすハーネス
    ├── shim/              # そのための最小の cf_math.h / stabilizer_types.h
    └── synth_log.py       # 合成の usddeck バイナリログ生成
```

### 3-4. 解析フロー

```
python3 analyze.py logs/log00 --geometry geometry/20260910_bs_geometry.yaml
```

の中でやっていること:

1. `parse_usdlog.parse()` でログを読む。時刻基準は各レコード先頭の `usecTimestamp()`。
2. `parse_usdlog.freshness()` が `upd<bs>` / `mask<bs>` から
   `valid[サンプル, BS, 素子]` を作り、**stale なサンプルを弾く**。
3. `geometry.load_geometry()` で BS ジオメトリを読む。
4. `crossing_beams.estimate_position_multi_bs()` で位置を算出
   （`--method firmware` でファーム互換の挙動にもできる）。
5. `yaw.solve_yaw()` で、4. の位置と IMU の roll/pitch を既知として yaw を解く。
   不動点反復は大域的でないので、先頭サンプルと途切れた直後だけ
   `solve_yaw_least_squares()` で大域解を作って初期値にしている。
6. `logs/<名前>_pose.csv` と `figs/<名前>_pose.png` を出力。
   `tofwall.*` との突き合わせは (h) の通り軌跡アライメント後の ATE / RPE で行う。

### 3-5. 移植の検証（実施済み）

`tests/test_port.py` は、ファームの `lighthouse_geometry.c` を `tests/shim/` の
最小ヘッダ（`cf_math.h` / `stabilizer_types.h` の代替）でホスト上に gcc でビルドし、
乱数ケースを C と Python の両方に通して数値を突き合わせる。

```
python3 tests/test_port.py --firmware ~/test/crazyflie_tmp/tmp/crazyflie-firmware-vl53l8cx
```

結果（全 8 項目 OK）:

- `lighthouseGeometryGetRay` / `GetPositionFromRayIntersection` /
  `IntersectionPlaneVector` / `GetSensorPosition` / `YawDelta` が C と一致
  （float32 と float64 の差の範囲）
- `predict_angles()` が `get_ray()` の逆変換になっている
- 合成した姿勢から位置を復元できる（誤差 < 1 μm）
- 不動点反復・最小二乗の両方が yaw を復元できる

さらに `tests/synth_log.py` が **ファームと同じバイナリ形式**（`usddeck.c:884-982`）で
合成ログを書き出すので、`parse_usdlog.py` → `analyze.py` を通しで確認できる。

BS 4 台・距離 4-5 m・スイープ角の雑音 0.5 mrad・BS ごとに 10% 欠損・30 Hz・20 秒での結果:

| 量 | 誤差 |
|---|---|
| 位置 | RMS **1.4 mm** / 最大 5.1 mm |
| yaw | RMS **5.0 deg** / バイアス -0.19 deg |

**②(a-1) の見積もりが数値で裏付けられた形**になっている。位置は ToF オドメトリの評価に
十分な真値になるが、**yaw はそのままでは真値として使えない**（評価対象と同程度以下の分解能）。
バイアスは無いので静止区間の平均や時間方向の平滑化と併用すること。

実機のログでは、これに加えて `lighthouse.method` を 0 にして
`lighthouse.x/y/z` も記録したログを 1 本取り、`analyze.py --compare-onboard` で
オンボードの計算結果と突き合わせて確認する。一致を確認したら `method` を 2 に戻す。

## ④ 層① を採用しなかった理由（参考）

4 BS 構成で、30 Hz 書き込みという条件下での比較。

前提: LH2 のロータは 50 rev/s・1 回転 2 ビーム → 1 素子 1 BS あたり 100 パルス/s。
4 BS × 4 素子で **1600 UART フレーム/s**。UART は 230400 baud（`lighthouse_core.c:539`）＝
**1920 フレーム/s が物理上限**で、既に 8 割方飽和している。

| | 層① UART フレーム | 層② スイープ角 | 層③ 派生値 |
|---|---|---|---|
| SD 書き込み量 | **~19 kB/s** | 4 kB/s | ~1 kB/s |
| 30 Hz 1 回あたり | ~640 B | 128 B + α | ~30 B |
| CPU が触る頻度 | **1600 Hz** | 30 Hz | 30 Hz |
| 追加実装 | **必要**（log 変数機構では扱えない） | log 宣言の昇格・追加のみ | 不要 |
| 8 kB バッファの猶予 | **~420 ms** | ~2 s | ~8 s |

重要な点として、**層②③ の「データ整形コスト」は追加コストではない**。
`lighthouseCoreTask` はログの有無に関わらず毎フレーム
「パルス復号 → `pulseProcessorApplyCalibration()` → `convertV2AnglesToV1Angles()`」まで
無条件に実行する（`lighthouse_core.c:568-590, 404-408`）。層② を記録する限界コストは
`usddeckWriteEventData()` の「log 変数のアドレスから 4 B memcpy を N 回」（`usddeck.c:512-532`）
だけで、30 Hz × 40 変数なら実質ゼロ。

逆に層① は「整形不要だから軽い」とはならず、**1600 Hz でリングバッファ push とミューテックス取得が
新規に発生する**。それが UART 受信を回している lighthouse core task の中に入るため、
フレーム取りこぼしのリスクに直結する。加えて (c) の SPI 競合下では、
SD の書き込みストール（数百 ms）で層① のバッファは容易に溢れる。

→ **層② を採用。層③ は層② から完全に再現できるため記録しない。**
