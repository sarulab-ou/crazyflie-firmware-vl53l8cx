# examples/demos 配下のアプリのビルド・書き込み・呼び出しの仕組み

`examples/demos/<app名>` (例: `app_cf_brushless_hover_move`) のようなアプリレイヤー
(app-layer) のデモを、そのフォルダ直下でビルド・書き込みしたときに、実際どういう
仕組みでデモのコードが動き出すのかをまとめる。一般的な out-of-tree ビルドの説明は
[out-of-tree build](oot.md)、app-layer 全般の説明は
[app layer](../userguides/app_layer.md) を参照。ここではそれを踏まえて、
「demo フォルダ直下で make したときに何が起きるか」を具体的に追う。
`app_cf_brushless_hover_move` 自体が具体的に何をするデモかは
[demo-app-cf-brushless-hover-move.md](demo-app-cf-brushless-hover-move.md) を参照。

## 1. リポジトリ直下でビルドした場合との違い

- リポジトリ直下の `Makefile` / `src/Kbuild` は `examples/` を一切参照しない。
- 直下用の defconfig (`configs/defconfig`, `configs/cf21bl_defconfig` 等) には
  `CONFIG_APP_ENABLE` の指定がなく、デフォルトでは app-layer 自体が無効。
- したがって **リポジトリ直下で `make` しても、`examples/demos` 配下のどのデモの
  コードもリンクされない**。素の標準ファームウェアができるだけ。

デモを機体に載せるには、必ずそのデモのフォルダに `cd` してから、そのフォルダの
`Makefile` でビルドする必要がある。

## 2. デモフォルダの構成 (out-of-tree ビルド)

`examples/demos/app_cf_brushless_hover_move/` を例にすると:

```
app_cf_brushless_hover_move/
├── Makefile     # CRAZYFLIE_BASE := ../../.. で本体を参照し、oot.mk を include
├── Kbuild       # obj-y += src/
├── app-config   # CONFIG_APP_ENABLE=y など、このアプリ専用の Kconfig 設定
└── src/
    ├── Kbuild                          # obj-y += app_cf_brushless_hover_move.o
    └── app_cf_brushless_hover_move.c   # appMain() を実装
```

- `Makefile` の `CRAZYFLIE_BASE := ../../..` は「自分の3階層上がリポジトリ
  ルート」という意味。`examples/demos/<app>/` はちょうど3階層目なので、
  `examples/demos/` 配下に置く前提の相対パスになっている
  (`examples/` 直下に置く既存アプリは `../..` の2階層)。
- ビルドは本体のソースツリーとは別に、このフォルダの中に `build/` が作られ、
  そこに `.config` や成果物が生成される。他のデモや本体ビルドとは独立している。
- `app-config` の内容は、本体の defconfig に対して
  `scripts/kconfig/merge_config.sh` でマージされる (`tools/make/oot.mk` 参照)。
  ここで `CONFIG_APP_ENABLE=y` を立てることで、初めて app-layer 機能が
  有効になったファームウェアがビルドされる。

## 3. ビルド・書き込み手順

```bash
cd examples/demos/app_cf_brushless_hover_move
make cf21bl_defconfig   # 対象プラットフォーム(この例では CF2.1 Brushless)のdefconfigを読み込み
make -j$(nproc)          # 本体ソース + このアプリの src/ を合わせてビルド
make cload                # ブートローダーモードのCrazyflieに書き込み
```

`make cload` で書き込むのは、**本体ファームウェア + このデモのコードを1本に
リンクした完全なファームウェアイメージ**。既存のファームウェア(標準アプリや
別のデモ)は上書きされて消える。複数のデモを同時には載せられず、都度入れ替える形。

## 4. ビルド成果物 (.bin) の場所

`make -j$(nproc)` でビルドすると、実際に `make cload` で書き込まれるバイナリは
デモフォルダ自身の中に生成される。`app_cf_brushless_hover_move` の場合:

```
examples/demos/app_cf_brushless_hover_move/build/cf21bl.bin   # ← cload / Slack共有等に使うのはこれ
examples/demos/app_cf_brushless_hover_move/build/cf21bl.elf   # デバッグ用 (gdb/openocd)
examples/demos/app_cf_brushless_hover_move/build/cf21bl.hex
examples/demos/app_cf_brushless_hover_move/build/cf21bl.map
```

ファイル名の `cf21bl` は `PROG ?= $(PLATFORM)` (`Makefile`) の `PLATFORM` から来ており、
`cf21bl_defconfig` で `CONFIG_PLATFORM_CF21BL=y` が設定されるため。別の defconfig
(例: `cf2_defconfig`) でビルドした場合は `cf2.bin` のように名前が変わる。

出力先が本体側の `build/` ではなく **デモフォルダ内の `build/`** になる理由:

- `tools/make/oot.mk` の `all:` ターゲットが
  `KBUILD_OUTPUT=$(OOT)/build` を指定してビルドする (`OOT` はデフォルトで
  デモフォルダ自身 = `$(PWD)`)。
- 本体 `Makefile` が include する `tools/kbuild/Makefile.kbuild` は Linux
  カーネル由来の Kbuild パターンを使っており、`KBUILD_OUTPUT` が設定されると
  そのディレクトリに `cd` して自分自身を再実行する (`sub-make` ターゲット)。
  そのため `$(PROG).bin` など相対パスの生成物は、実行時のカレントディレクトリ
  である `$(KBUILD_OUTPUT)` = デモフォルダ内の `build/` に生成される。

## 5. 書き込み後、デモのコードはどう呼ばれるか

`appMain()` は直接 `main()` のように呼ばれるのではなく、**app-layer という
フック機構を通して専用の FreeRTOS タスクとして起動**される。

```
起動 (systemInit(), src/modules/src/system.c)
  │
  ├─ #ifdef CONFIG_APP_ENABLE   … app-config で有効化されている場合のみ
  │     appInit()                (src/modules/src/app_handler.c, weak関数)
  │        │
  │        └─ STATIC_MEM_TASK_CREATE(appTask, ...)
  │              優先度 = CONFIG_APP_PRIORITY (app-configで指定, 例: 1)
  │              スタック = CONFIG_APP_STACKSIZE (app-configで指定, 例: 350)
  │              │
  │              └─ appTask() が新規タスクとして走り出す
  │                    systemWaitStart()   … 全体の初期化完了を待つ
  │                    appMain()            … ★ここでデモ側のコードに処理が渡る
  │                    while(1) vTaskDelay(portMAX_DELAY);  … appMain が戻ったら以後は寝るだけ
```

要点:

- `src/modules/interface/app.h` は `void appMain();` を宣言しているだけで、
  本体側は実装を持たない。
- 実装は、OOT ビルドでリンクされるデモ側の `src/app_cf_brushless_hover_move.c`
  の `appMain()` (159行目) が使われる。`src/Kbuild` の
  `obj-y += app_cf_brushless_hover_move.o` によって、このファイルが
  ビルド・リンク対象に含まれる。
- `CONFIG_APP_ENABLE` が有効なビルドでのみ `src/modules/src/Kbuild` の
  `obj-$(CONFIG_APP_ENABLE) += app_handler.o` が効き、`appHandler.c` 自体が
  リンクされる。つまりデモフォルダの `app-config` がこの一連の仕組み全体の
  スイッチになっている。
- `appMain()` は普通は戻らない前提の関数で、内部で `while(1)` 相当のループを
  回しながら状態遷移 (例: 離陸→ホバー→前進→ホバー→着陸のFSM) を実行し続ける。
  電源投入と同時にこのタスクは生成されるが、実際にミッションが動き出すかは
  アプリ次第 (`app_cf_brushless_hover_move` の場合は `hoverMove.start`
  パラメータのセット、または上向きセンサーへの手かざしがトリガー)。

## 6. まとめ

| 動作 | 何が起きるか |
| --- | --- |
| リポジトリ直下で `make` | app-layer 無効の標準ファームウェアのみビルド。デモは一切含まれない |
| `examples/demos/<app>/` 直下で `make ...` | 本体 + そのデモの `appMain()` を1本にリンクしたファームウェアをビルド。成果物は `examples/demos/<app>/build/<platform>.bin` (elf/hex/mapも同フォルダ) |
| `make cload` | `build/<platform>.bin` を使って機体の既存ファームウェアを丸ごと上書き |
| 電源投入後 | `CONFIG_APP_ENABLE` により `appInit()` → 専用タスク生成 → `appMain()` 呼び出しが自動で発生 |
| デモの実際の飛行開始 | `appMain()` 内のロジック次第 (パラメータ経由 / センサートリガー / 無線コマンド等、アプリごとに異なる) |
