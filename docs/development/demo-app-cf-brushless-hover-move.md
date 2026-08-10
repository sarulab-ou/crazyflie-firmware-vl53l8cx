# app_cf_brushless_hover_move デモの詳細

`examples/demos/app_cf_brushless_hover_move/src/app_cf_brushless_hover_move.c`
の中身を詳しく解説する。ビルド方法・書き込み後の呼び出し経路(`appMain()`が
どう起動されるか)は [demo-app-build-and-invocation.md](demo-app-build-and-invocation.md)
を参照。ここでは **このアプリが実際に何をする、どう動くコードなのか** に絞る。

## 1. 一言でいうと

Crazyflie Brushless (CF2.1BL) + vl53l8cx デッキ(9方向レンジセンサー) +
Flow deck v2 の組み合わせで、**無線接続なしでも完結する自律ミッション**を
実行するapp-layerアプリ。

```
アーム → 0.3m まで離陸 → その場でホバー(5s) → 前方に0.5m移動 → 移動先でホバー(5s) → 着陸
```

をオンボードだけで実行する。`app_cf_brushless_circle.c`という円軌道版アプリの
"円移動フェーズ"を"前進move"に置き換えた派生版で、アーム処理・uSDロギング・
トリガー機構はそちらと共通(ファイル冒頭コメントに明記)。

## 2. 前提となるハードウェア構成

- **Crazyflie 2.1 Brushless** (CF21BL プラットフォーム)
- **Flow deck v2** (`deck.bcFlow2` パラメータで検出。位置推定 x/y/z に必須)
- **vl53l8cx デッキ (9センサー版)**: `vl53l8cx.s0`〜`s8` の9方向レンジ。
  上向き(`s8`)はハンドトリガー、他は主にuSDへの記録用(このアプリ自体は
  障害物回避などには使っていない)
- **推定器はKalmanフィルタに強制設定** (`appMain()`冒頭で
  `paramSetInt(idEstimator, 2)`)

センサー方向の対応表 (`sdcard/readme.md` より):

| index | 方向 |
|-------|------|
| s0 | front-left |
| s1 | center |
| s2 | front-right |
| s3 | front-up |
| s4 | front-down |
| s5 | left |
| s6 | right |
| s7 | back |
| s8 | up |

## 3. 起動シーケンス (`appMain()` の頭)

1. `vTaskDelay(M2T(3000))` で3秒待機 (他モジュールの初期化が落ち着くのを待つ)
2. 必要な log/param の変数IDを一括取得
   (`stateEstimate.x/y/z`, `stabilizer.yaw`, `sys.isTumbled`, `vl53l8cx.s8`,
   `deck.bcFlow2`, `usd.logging`, `stabilizer.estimator`)
3. 推定器をKalman (`estimator = 2`) に強制設定
4. **その後は電源が入っている限り、50Hz (`loopDt_ms = 20ms`) の無限ループで
   ステートマシンを回し続ける。** これがミッション本体。

電源投入直後は自動でミッションは始まらない(`startMission = 0`)。トリガーが
かかるまでは何もせずアイドル状態のまま待機し続ける。

## 4. トリガー方法 (ミッション開始のきっかけ)

2通りあり、どちらもアイドル状態(`APP_IDLE`)でのみ受け付ける:

1. **パラメータ経由**: `hoverMove.start` を `1` にセット (cfclientやPythonから、
   無線/USB経由)
2. **ハンドトリガー(PCリンク不要)**: 上向きセンサー`vl53l8cx.s8`に
   `hoverMove.handMm` (既定300mm) 以内の距離で `hoverMove.handHoldMs`
   (既定500ms) 以上、手をかざし続ける

どちらの方法でも内部的には `startMission = 1` がセットされ、以降のステートマシンが
動き出す。

## 5. ステートマシン (`appState_t`) 全体像

```
APP_IDLE
   │  (start param か ハンドトリガー)
   ▼
APP_WAIT_FOR_DECK
   │  (Flow deckが安定して500ms検出され続けたら)
   ▼
APP_PREARM
   │  (1000ms 待機)
   ▼
APP_ARMING
   │  (supervisorRequestArming(true) を出し続け、armed成立を待つ。3秒でタイムアウト)
   ▼
APP_TAKEOFF
   │  (3秒かけて0m→takeoffHeightまで直線的に上昇)
   ▼
APP_HOVER1
   │  (離陸地点で hoverTimeS 秒ホバー)
   ▼
APP_MOVE
   │  (moveTimeS 秒かけて前方に moveDistanceM 移動)
   ▼
APP_HOVER2
   │  (移動先で hoverTimeS 秒ホバー)
   ▼
APP_LAND
   │  (一定速度で降下、landCutoffM を下回るかlandMaxTime_s経過でモーターカット)
   ▼
APP_IDLE (自動的に戻る)
```

各状態は `switch (appState)` の1ケースとして実装されており、ループ1周
(=20ms)ごとに現在の状態の処理を1回実行する、という単純な有限状態機械 (FSM)。

### 各状態の詳細

| 状態 | やること | 次の状態への遷移条件 |
|---|---|---|
| `APP_IDLE` | setpointを止める。uSDロギングをOFFに。ハンドトリガー検出処理 | `startMission==1` になったら `APP_WAIT_FOR_DECK` |
| `APP_WAIT_FOR_DECK` | 何もせず待機、setpointは停止のまま | Flow deck (`deck.bcFlow2`) が500ms連続で検出できたら `APP_PREARM`。`startMission`が0に戻ったら`APP_IDLE`へ |
| `APP_PREARM` | 1秒間の助走待機 | 1000ms経過で `APP_ARMING`。デッキ消失で`APP_WAIT_FOR_DECK`へ戻る |
| `APP_ARMING` | 毎ループ `supervisorRequestArming(true)` を送り続ける | `supervisorIsArmed()`がtrueになったら現在位置を`takeoffX/Y`として記録し`APP_TAKEOFF`へ。3秒たっても armed にならなければ諦めて`APP_IDLE`(ミッション中止) |
| `APP_TAKEOFF` | uSDロギング開始。3秒かけて z を 0→`takeoffHeight`まで線形に上げる絶対位置setpointを送る | 3秒経過(a>=1.0)で`APP_HOVER1`。デッキ消失や`startMission`解除で即`APP_LAND`へ緊急移行 |
| `APP_HOVER1` | 離陸地点 (`takeoffX, takeoffY, takeoffHeight`) を維持するsetpointを送り続ける | `hoverTimeS`秒(既定5秒)経過で `moveX = takeoffX + moveDistanceM` を計算し `APP_MOVE`へ |
| `APP_MOVE` | `moveTimeS`秒かけて x を `takeoffX → takeoffX+moveDistanceM` へ線形に移動するsetpointを送る (y/zは固定) | 移動完了(a>=1.0)で `APP_HOVER2`へ |
| `APP_HOVER2` | 移動先 (`moveX, moveY, takeoffHeight`) を維持 | `hoverTimeS`秒経過で `landX/Y = moveX/Y` を設定し `APP_LAND`へ |
| `APP_LAND` | `landX, landY` 位置固定・z方向のみ速度制御(`-landSpeed` = 0.15m/s降下)のsetpointを送る | `estZ <= landCutoffM` (4cm) または `landMaxTime_s`(6秒)経過でモーター停止(`supervisorRequestArming(false)`)、uSDロギング停止、`APP_IDLE`へ |

各フェーズの移動は「毎ループその場の推定値を再サンプルする」のではなく、
**状態突入時に1回だけ固定した基準点 (`takeoffX/Y`, `moveX/Y`, `landX/Y`) から
時間で線形補間した目標値**を送り続ける方式(コメントにも明記: ホールド中に
推定値を使うと復元力が働かず位置がドリフトするため)。

## 6. 安全機構

- **転倒検知 (`sys.isTumbled`)**: 飛行中の状態(`APP_ARMING`〜`APP_LAND`)であれば
  毎ループチェックし、転倒を検知した瞬間にuSDロギング停止・setpoint停止・
  `supervisorRequestArming(false)`・`APP_IDLE`へ強制リセットする(ループの
  この時点で`switch`文自体をスキップして`continue`)。
- **Flow deckロスト時のフェイルセーフ**: `APP_TAKEOFF`〜`APP_HOVER2`の各状態で
  毎回 `positioningInit` (=`deck.bcFlow2`) をチェックしており、途中でデッキが
  外れる/認識が切れると即座に現在位置基準の`APP_LAND`へ移行し、安全側に着陸する。
- **アーミングタイムアウト**: `APP_ARMING`で3秒以内にarmedにならない場合は
  ミッション自体を中止してアイドルに戻る(無限リトライしない)。
- **ミッション中断**: 飛行中に`startMission`が0に戻された場合(外部から明示的に
  クリアされた場合)も、その場から`APP_LAND`に移行して安全に降ろす。
- **着地の二重条件**: 高度がしきい値を下回る、または最大待ち時間を超える、
  いずれか早い方でモーターカットするため、センサー誤差で高度が
  下がりきらない場合でも6秒で強制終了する。

## 7. チューナブルパラメータ (`PARAM_GROUP_START(hoverMove)`)

無線/USB経由、再書き込み不要でランタイムに変更可能:

| パラメータ | 既定値 | 意味 |
|---|---|---|
| `hoverMove.start` | 0 | 1にするとミッション開始トリガー |
| `hoverMove.takeoffH` | 0.3 m | 離陸高度 |
| `hoverMove.moveDistM` | 0.5 m | 前進移動距離 |
| `hoverMove.hoverTimeS` | 5.0 s | 各ホバーフェーズの時間 |
| `hoverMove.moveTimeS` | 5.0 s | 前進移動にかける時間 |
| `hoverMove.handMm` | 300 mm | ハンドトリガー検出距離 |
| `hoverMove.handHoldMs` | 500 ms | ハンドトリガーの保持時間 |
| `hoverMove.landSpeed` | 0.15 m/s | 着陸降下速度 |
| `hoverMove.landCutoffM` | 0.04 m | モーターカットする高度 |

## 8. ログ変数 (`LOG_GROUP_START(hoverMove)`)

デバッグ・監視用に無線/USB経由でリアルタイム参照できる:

| ログ | 型 | 内容 |
|---|---|---|
| `hoverMove.state` | uint8 | 現在の`appState`(数値、上記enumの並び順) |
| `hoverMove.isTumbled` | uint8 | `sys.isTumbled`のミラー |
| `hoverMove.usdOn` | uint8 | uSDロギングが今アクティブか |
| `hoverMove.flowOk` | uint8 | Flow deck検出状態 |
| `hoverMove.armed` | uint8 | アーム状態 |
| `hoverMove.handNear` | uint8 | 上向きセンサーへの手かざしを検出中か |
| `hoverMove.estX/estY/estZ` | float | 推定位置 |
| `hoverMove.yawDeg` | float | 推定ヨー角 |
| `hoverMove.phaseElapsedS` | float | 現フェーズ(ホバー/移動)の経過時間 |

## 9. uSDカードロギング

`usd.logging` パラメータをこのアプリが自動制御する(離陸開始時に1、着陸完了・
中断・クラッシュ時に0)。物理的には1ファイルに2つの論理ストリームを
多重化して記録し、`sd_log_to_csv.py`が展開時に自動で2つのCSVに分離する
(詳細は`sdcard/readme.md`):

- **`fixedFrequency`ストリーム**: `vl53l8cx.s0`〜`s8`の9方向レンジ値。
  同期スタビライザーフックで50Hz固定周期記録 (`sdcard/config.txt`で設定)。
- **`pose`ストリーム**: `stateEstimate.x/y/z`, `stabilizer.roll/pitch/yaw`。
  このアプリの`appMain()`ループ内で毎ループ明示的に
  `eventTrigger(&eventTrigger_pose)` を呼んで記録(実質約50Hz、
  `usd.logging`がOFFのときはno-opなので呼びっぱなしでも問題ない実装)。

`sdcard/config.txt` を microSD カードの**ルート**に `config.txt` という
名前でコピーしておく必要がある(アプリフォルダに置いてあるだけでは反映されない)。

## 10. 実装上の細かい注意点

- `setAbsSetpoint()` は x/y/z すべて絶対位置モード(`modeAbs`)、yawも絶対角度
  固定(常に0度)で送る単純な実装。
- `setLandDescentSetpoint()` だけ z を速度モード(`modeVelocity`)にして
  一定速度で降下させ、x/yは着地点に固定する。位置ランプ(時間で高度を決め打ち)
  ではなく速度指定にしているのは、着地の瞬間の衝撃を和らげるための工夫
  (コメントに理由が明記されている)。
- `commanderSetSetpoint(&setpoint, 3)` の第2引数`3`はコマンダーの
  優先度/ソースID。app-layerからの直接setpoint送信であることを示す。
- ミッション全体を通してyaw制御は常に0度固定で、機体の向き変更は行わない。
- デフォルトでは電源投入だけで自動飛行はしない(`startMission = 0`)。ファイル
  冒頭コメントに「起動時に無条件で飛ばす設定に変える場合は、GUIから止める
  手段がなくなるため周囲の安全を確保してから」との注意書きがある。

## 11. まとめ図

```
[電源ON]
   │
   ▼
appMain() 開始 (3秒待機 → 変数ID取得 → Kalman推定器強制)
   │
   ▼
50Hzループ: FSM (APP_IDLE ⇄ トリガー待ち)
   │  hoverMove.start=1 または 手かざし500ms
   ▼
WAIT_FOR_DECK → PREARM → ARMING → TAKEOFF
   │
   ▼
HOVER1(5s) → MOVE(前進0.5m/5s) → HOVER2(5s)
   │
   ▼
LAND (0.15m/s降下 → 4cm以下 or 6秒でモーターカット)
   │
   ▼
IDLE に自動復帰 (再トリガー可能)
```
