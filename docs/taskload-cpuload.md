# taskload.cpuLoad の計算方法

`taskload.cpuLoad` は `src/modules/src/taskload.c` で計算され、LOG 変数として公開されている CPU 使用率 [%] です。
microSD デッキへの記録は `tools/usdlog/config_taskload.txt` などの config.txt にこの変数を書くことで行います。

## 結論

- **「実行時間 × 周波数」の見積もりではありません。** FreeRTOS がコンテキストスイッチのたびに記録している、各タスクの**実測の累積実行時間 [us]** を使っています。
  ある区間にタスクが何回起動されても、その区間で実際に CPU を占有した時間の合計が積算されるので、周波数は自動的に反映されます。
- **すべてのタスクが含まれます。** cpuLoad は「IDLE タスク以外が CPU を使っていた時間の割合」として計算するので、`entries[]` に登録されていないタスク（タイマーデーモン `Tmr Svc` など）も含まれます。

```
cpuLoad [%] = 100 × (区間の経過時間 − 区間中の IDLE タスクの実行時間) / 区間の経過時間
```

## 仕組み

### 1. 実行時間カウンタ（FreeRTOS 側）

`src/config/config.h` で FreeRTOS の実行時間統計を有効にしています。

```c
#define configGENERATE_RUN_TIME_STATS 1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() usecTimerInit()
#define portGET_RUN_TIME_COUNTER_VALUE() usecTimestamp()
```

`usecTimestamp()` は TIM7 で作った 1 MHz（1 us 分解能）のタイムスタンプです（`src/hal/src/usec_time.c`）。

FreeRTOS は `vTaskSwitchContext()`（`vendor/FreeRTOS/tasks.c`）で、タスクが CPU から外れるたびに次の処理を行います。

```c
ulTotalRunTime = portGET_RUN_TIME_COUNTER_VALUE();      // 現在時刻 [us]
pxCurrentTCB->ulRunTimeCounter += ulTotalRunTime - ulTaskSwitchedInTime;
ulTaskSwitchedInTime = ulTotalRunTime;
```

つまり各タスクの `ulRunTimeCounter` は「起動からそのタスクが実際に走っていた時間の合計 [us]」です。
プリエンプトされていた時間やブロックしていた時間は含まれません。

### 2. 周期サンプリング（taskload.c 側）

`taskLoadInit()`（`system.c` から呼ばれる）で、周期 `periodMs`（デフォルト 1000 ms）の FreeRTOS ソフトウェアタイマーを作ります。
タイマーのコールバック `timerHandler()` では次の処理を行います。

1. `uxTaskGetSystemState()` で全タスクの `ulRunTimeCounter` と、現在時刻 `totalRunTime`（= `usecTimestamp()`）を取得する。
2. 登録済みの各タスクについて、前回からの差分 `delta = ulRunTimeCounter − prevRunTime` を求め、`taskload.<TASK>` に入れる。
3. IDLE タスクの差分を `idleDelta` として保存する。
4. `totalDelta = totalRunTime − prevTotalRunTime`（前回サンプルからの経過時間 [us]）を求める。
5. `cpuLoadPct = 100 × (totalDelta − idleDelta) / totalDelta` を計算する。

```c
uint32_t totalDelta = totalRunTime - prevTotalRunTime;
prevTotalRunTime = totalRunTime;
if (totalDelta > 0) {
  cpuLoadPct = 100.0f * (float)(totalDelta - idleDelta) / (float)totalDelta;
}
```

`totalRunTime` は「時刻」なので、`totalDelta` はタスクごとの合計ではなく**経過時間そのもの**です。
このため、IDLE 以外の部分には登録済み・未登録を問わずすべてのタスク、割り込み、カーネルのオーバーヘッドが入ります。

## 関連する LOG 変数

| 変数 | 型 | 意味 |
|---|---|---|
| `taskload.cpuLoad` | float | CPU 使用率 [%]。上の式で計算 |
| `taskload.total` | uint32 | `entries[]` に**登録済みのタスクだけ**の実行時間の合計 [us]（IDLE を含む） |
| `taskload.period` | uint32 | 実測のサンプリング区間 [us]。`timerHandler` 内の `usecTimestamp()` の差分 |
| `taskload.<TASK>` | uint32 | そのタスクが区間中に使った CPU 時間 [us] |
| `taskload.IDLE` | uint32 | IDLE タスクの区間中の実行時間 [us] |

タスクごとの使用率は `taskload.<TASK> / taskload.period` で求められます（デフォルトの 1000 ms 周期なら、値はそのまま「1 秒あたりの CPU 時間 [us]」です）。

`period − total` は、登録していないタスク（`Tmr Svc` など）の実行時間とカーネルのオーバーヘッドの目安になります。

### 検算

```
cpuLoad ≈ 100 × (period − IDLE) / period
```

`period` と cpuLoad の `totalDelta` はどちらも約 `periodMs` ごとの `usecTimestamp()` の差分ですが、取得するタイミングが少し違うので、完全には一致しません。

## 注意点・制約

- **割り込みの時間は、その時に走っていたタスクに計上されます。**
  IDLE 中に起きた割り込み（モーター PWM、センサー DMA、TIM7 など）は IDLE の時間として数えられるので、そのぶん cpuLoad は**低めに出ます**。
  逆に、あるタスクの実行中に起きた割り込みはそのタスクの時間に入ります。
- **IDLE フック (`vApplicationIdleHook`, `system.c`) の処理は IDLE として数えられます**（ウォッチドッグのリセットなど）。
- **更新は `periodMs` ごと（デフォルト 1 Hz）です。** usdlog を 10 Hz で記録すると同じ値が約 10 サンプル続くので、解析では値が変わった行だけを使うか、`period` を見て区間を区切ってください。
- **最初のサンプルは起動から最初のタイマー発火までの区間です**（`prevTotalRunTime = 0` から始まるため）。初期化中の負荷が入るので、解析では捨てるのが無難です。
- **`periodMs` パラメータは起動時にしか反映されません。** タイマーは `taskLoadInit()` で作ったきりなので、実行中に値を変えても周期は変わりません（persistent パラメータでもありません）。周期を変えるにはソースのデフォルト値を変えてください。
- **タイマーコールバックは `Tmr Svc` タスク（優先度 `configTIMER_TASK_PRIORITY = 1`）で動きます。** 高負荷時には発火が遅れて区間が長くなることがありますが、計算には実測の経過時間を使うので、比率としては正しいままです。
- **32 bit のラップアラウンド。** `usecTimestamp()` は 64 bit ですが、FreeRTOS の実行時間カウンタは 32 bit なので、約 71.6 分（2^32 us）で一周します。
  差分は符号なし減算なので区間をまたいでも正しく出ます。ただし FreeRTOS は `ulTotalRunTime > ulTaskSwitchedInTime` のときだけ加算するので、ラップした瞬間の 1 スライスぶんだけ計上漏れが起きます（実用上は無視できます）。
- **タスク数の上限。** `uxTaskGetSystemState()` に渡すバッファは `TASK_MAX_COUNT = 48` です。実際のタスク数がこれを超えると取得に失敗し（`ASSERT` が発生）、IDLE が見つからないと `idleDelta = 0` になって cpuLoad が 100% と表示されます。
- **`taskload.<TASK>` はタスク名の完全一致で探しています**（`entries[].taskName` と `config.h` の `*_TASK_NAME`）。名前が違ったり、そのデッキが載っていなかったりするタスクは 0 のままです。cpuLoad 自体はこの影響を受けません。

## 関連ファイル

- `src/modules/src/taskload.c` — 計算と LOG/PARAM の定義
- `src/modules/interface/taskload.h`
- `src/modules/src/system.c` — `taskLoadInit()` の呼び出し、`vApplicationIdleHook`
- `src/config/config.h` — 実行時間統計の設定
- `src/hal/src/usec_time.c` — `usecTimestamp()`（TIM7, 1 us）
- `vendor/FreeRTOS/tasks.c` — `vTaskSwitchContext()`, `uxTaskGetSystemState()`
- `tools/usdlog/config_taskload.txt` — SD 記録用の設定例
- `docs/feat#6/experiment.md` — この値を使う実験計画
