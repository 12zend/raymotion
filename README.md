# raymotion

C++17へコンパイルするシーン記述言語とCPUパストレーシングCLIです。
固定シーン・OBJ・カメラ軌道は同梱しません。

## ビルドとインストール

必要: CMake 3.16以上、C++17コンパイラ、Python 3.9以上。
MP4出力のみFFmpeg（libx264対応）が必要です。macOS / Linuxを対象にしています。

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$HOME/.local"
export PATH="$HOME/.local/bin:$PATH"
```

インストールせず `./src/cli/raymotion` でも実行できます。

## miseからのインストール

`.github/workflows/release.yml` は `v*` タグのpushでOS・CPU別の配布アーカイブを
GitHub Releasesに生成します。CLI、C++ヘッダー、コンパイル済みエンジンを含みます。
リポジトリの公開・タグのpush後に、`OWNER` を実際の所有者へ置き換えて実行します。
この作業コピーにはGitリモートが設定されていないため、配布URLは未確定です。

```sh
mise use -g github:OWNER/raymotion@latest
raymotion --version
```

miseの[GitHub backend](https://mise.jdx.dev/dev-tools/backends/github.html)を利用します。
利用先にもPython 3とC++17コンパイラが必要です。Release版はエンジンの再ビルド不要です。

## CLI

```sh
raymotion init my-project
cd my-project
raymotion build
raymotion export out.png -w 1280 -h 720 -s 64
raymotion export out.mp4 --width 1280 --height 720 --framerate 30 --sample 16
```

`init` は `assets/` と `main.ray` を作成し、既存の `main.ray` を上書きしません。
引数省略時は現在のディレクトリに生成します。
`build` は現在のディレクトリの `main.ray` を読み、`.raymotion/build/main.cpp` と
実行ファイル `main` を生成します。`export` は必ずこのビルドを行ってから実行します。

| オプション | デフォルト | 意味 |
| --- | --- | --- |
| `-w / --width` | 640 | 横ピクセル数 |
| `-h / --height` | 360 | 縦ピクセル数 |
| `-f / --framerate` | 30 | 動画FPS・プログラム内の`framerate` |
| `-s / --sample` | 16 | 1ピクセルのサンプル数 |

ヘルプは `raymotion export --help`。`-h` は高さです。
MP4のサイズは偶数が必要です。出力先の親フォルダは自動生成します。
PNGは `object.render()` 1回、MP4は1回以上必要です。
動画時間は `render()` の実行回数 ÷ FPSです。拡張子は大文字も受け付けます。

## 言語

`main.ray` はC++17の文を記述するスクリプトです。エントリーポイントは自動生成します。
変数、条件、ループ、ラムダ、標準ライブラリが使えます。`#include` はファイル先頭に置きます。
通常の自由関数・クラス等の大きな定義はヘッダーに置いてincludeしてください。
C++と同じ権限で実行されるため、自分が信頼するソースをビルドしてください。

```cpp
void example = object.init("assets/model.obj");
object.push(example, {0, 0, 3}, {0, 45, 0}, {1, 1, 1},
            {0.8, 0.3, 0.1}, {0, 0, 0}, 1, 0.5, 0);
object.render();
```

`void name = object.init(...)` はコンパイラによってモデルハンドルの`auto`宣言に変換されます。
文字列・コメント内は変換しません。残りの構文はC++コンパイラで型検査します。
診断には`main.ray`の行番号を出します。

`object.init` はOBJを読み込みます。同じパスは一度だけ読み込み、複数インスタンスで共有します。
OBJのZ座標と面の巻き順は既存エンジンに合わせて反転されます。

```cpp
object.push(example);                 // すべて既定値
object.push(example, {0, 0, 3});      // 位置のみ
```

| 引数（順番） | 型 | デフォルト |
| --- | --- | --- |
| model | モデルハンドル | 必須 |
| position | Vec3 | `{0,0,0}` |
| rotation | Vec3 | `{0,0,0}`、度、X→Y→Z順 |
| scale | Vec3 | `{1,1,1}` |
| albedo | Vec3 | `{1,1,1}` |
| emission | Vec3 | `{0,0,0}` |
| refract | double | `1`（屈折率） |
| rougth | double | `0.5`（要求仕様の綴り） |
| metallic | double | `0` |

末尾引数の省略は生成C++のAPI既定引数で補完されます。
途中だけ省略する場合はその引数の既定値を明示してください。
`Vec3{x,y,z}` または `{x,y,z}` が使えます。

カメラ初期値は原点、回転ゼロ、+Z方向、FOV 60度です。
`camera.x/y/z`、`camera.dirx/diry/dirz`（度）、`camera.fov`で指定できます。
CLIで指定した`width`、`height`、`sample`、`framerate`は読み取り専用変数です。

### 動画

```cpp
void example = object.init("assets/model.obj");
for (int frame = 0; frame < framerate * 2; ++frame) {
    double time = double(frame) / framerate;
    object.push(example, {0, 0, 3}, {0, time * 90, 0}, {1,1,1},
                {1,1,1}, {0.2,0.2,0.2});
    object.render();
}
```

各`render()`が1フレームを出力し、pushキューを空にします。
次フレームに表示するオブジェクトは再度pushしてください。モデルとカメラは保持されます。
BVHは描画直前に生成します。同じモデル順序・三角形数なら境界を下から更新（refit）し、
走査用float/BVH4キャッシュも更新します。構造変更時と32フレームごとにSAHで再構築します。
後者は長時間の移動による木の品質劣化を抑えるためです。性能比較のベンチマークは未実施です。

## ディレクトリ

```text
src/cli/                 CLI・.rayコンパイラ
src/engine/              OBJ・BVH・パストレーサー・画像出力
include/raymotion/      C++ランタイムと公開ヘッダー
tests/                   コンパイラ・CLI・BVH検証
.github/workflows/      テスト・Release配布
```

インストール済みCLIのE2E検証: `python3 tests/smoke_export.py /absolute/path/to/bin/raymotion`。
