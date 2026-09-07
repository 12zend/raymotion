# raymotion

C++17へコンパイルするシーン記述言語と、Metal GPU専用パストレーシングCLIです。
固定シーン・OBJ・カメラ軌道は同梱しません。

## ビルドとインストール

必要: CMake 3.16以上、C++17コンパイラ、Python 3.9以上。
MP4出力のみFFmpeg（libx264対応）が必要です。macOSを対象にしています。

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$HOME/.local"
export PATH="$HOME/.local/bin:$PATH"
```

インストールせず `./src/cli/raymotion` でも実行できます。

## miseからのインストール

`.github/workflows/release.yml` は `v*` タグのpushでmacOSのアーキテクチャ別の配布アーカイブを
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
実行ファイル `main` を生成します。`export` はビルドの更新を確認してから実行します。ソース・依存ヘッダー・エンジン・コンパイラが同じ場合は実行ファイルを再利用します。

| オプション | デフォルト | 意味 |
| --- | --- | --- |
| `-w / --width` | 640 | 横ピクセル数 |
| `-h / --height` | 360 | 縦ピクセル数 |
| `-t / --time` | 10 | 動画の最大時間（秒、小数可） |
| `-f / --framerate` | 30 | 動画FPS・プログラム内の`framerate` |
| `-s / --sample` | 16 | 1ピクセルのサンプル数 |

ヘルプは `raymotion export --help`。`-h` は高さです。
MP4のサイズは偶数が必要です。出力先の親フォルダは自動生成します。
PNGは `object.render()` 1回、MP4は1回以上必要です。
動画時間は `render()` の実行回数 ÷ FPSです。拡張子は大文字も受け付けます。

## Metal GPUによる高速化

描画はMetal専用です。GPUを利用できない場合はエラーになります。
`--device` と `RAYMOTION_DEVICE` による選択、CPU専用ビルドは廃止しました。
C++17コンパイラはシーンプログラムのビルドに引き続き必要です。

```sh
raymotion export out.png -w 1280 -h 720 -s 128
raymotion export out.mp4 -w 1280 -h 720 -s 32
```

シェーダーはライブラリへ埋め込み、初回描画時にGPU用パイプラインをコンパイルします。

対応GPUではMetalのレイトレーシングAPIを使用し、非対応GPUではMetal compute上で
スタック不要のBVH走査を実行します。MetalレイトレーシングAPIはmacOS 11以降が対象です。
M1でも動作し、レイトレーシング専用ハードウェアの搭載は必須ではありません。
1画素を8または32レーンで分担し、GPU向け32bit乱数生成とfloat演算を使います。
拡散反射、GGX金属、粗い／滑らかな屈折、発光、直接照明・MIS、クランプ、適応サンプリングを
実装しています。

GPUバッファとパイプラインはフレーム間で再利用します。頂点が変化しないフレームでは
Metal加速構造を再利用し、同じ三角形数で頂点が変化した場合はrefitします。
31回のrefit後、または三角形数の変更時に再構築します。
OBJ読み込み・オブジェクト変換・CPU側BVH更新・画像保存・FFmpeg処理はCPUで実行します。
そのため、小さな画像や低サンプル数、コンパイル・動画エンコードが支配的な処理では
全体の短縮率が小さくなる場合があります。

検証:

```sh
RAYMOTION_REQUIRE_METAL=1 ctest --test-dir build --output-on-failure
python3 tests/smoke_export.py /absolute/path/to/bin/raymotion
```

Metalテストは空シーン、材質、再現性、透過の期待値、適応サンプリング、端数サイズ、
34フレームの更新を検証します。
`RAYMOTION_METAL_TRAVERSAL=software` を指定すると、Metal compute版BVHも検証できます。

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

`object.init(objPath, mtlPath = "")` はOBJと、省略可能なMTLを読み込みます。
同じOBJ・MTLの組み合わせは一度だけ読み込み、複数インスタンスで共有します。

```cpp
void model = object.init("assets/model.obj", "assets/model.mtl");
object.push(model, {0, 0, 3});
```

OBJの `usemtl` に従って、MTLの `Kd`（albedo）、`Ke`（emission）、
`Ni`（refract）、`Pr`（rougth）、`Pm`（metallic）を適用します。
`Pr` がない場合、`Ns` は `sqrt(2 / (Ns + 2))` でroughnessへ変換します。
このレンダラーは屈折率が1.0001を超えると誘電体として扱います。
`map_Kd` の画像パスはMTLのあるフォルダから解決します。PNG/JPEG/TGA/BMP/PPMなどを読み込み、
sRGBから線形RGBへ変換して、OBJのUVでMetalに反映します。
画像は繰り返し・最近傍サンプリングでalbedoに乗算します。UVのない面には画像を適用しません。
スペースを含む画像パスにも対応します。`map_Kd` のオプション、透過、その他のマップは未対応です。
MTLは第2引数で明示指定します（`mtllib` の自動読み込みは行いません）。
指定したMTLや画像を読み込めない場合はエラーにします。

`push` のマテリアル引数を省略するとモデルの値を使い、明示指定するとその項目を上書きします。
MTLなしのモデルは下表の従来の既定値になります。
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
| alpha | double | `1`（不透明） |

`alpha` は `0` で完全透明、`1` で不透明、`0.5` で半透明です。範囲外は0〜1に制限します。
Metalで面ごとの透過を描画し、影と発光にも反映します。屈折は `refract` で別途指定します。
閉じた形状では手前と奥の面それぞれに適用されます。

```cpp
object.push(model, {0,0,3}, {}, {1,1,1}, {-1,-1,-1}, {-1,-1,-1}, {}, {}, {}, 0.5);
```

末尾引数の省略は生成C++のAPI既定引数で補完されます（マテリアルはモデルから継承）。
途中だけ省略する場合はその引数の既定値を明示してください。
`Vec3{x,y,z}` または `{x,y,z}` が使えます。

カメラ初期値は原点、回転ゼロ、+Z方向、FOV 60度です。
`camera.set(position, rotation, fov);` でまとめて設定できます。
`object.push` と同様に末尾引数を省略でき、省略した値は以前の値ではなく既定値になります。

```cpp
camera.set();                          // 原点、回転ゼロ、FOV 60度に戻す
camera.set({0, 1, -3});                // 位置のみ（回転ゼロ、FOV 60度）
camera.set({0, 1, -3}, {0, 15, 0});    // 位置と回転（度）
camera.set({0, 1, -3}, {0, 15, 0}, 45); // すべて指定
camera.set({}, {}, 45);               // 原点、回転ゼロ、FOV 45度
```

`camera.x/y/z`、`camera.dirx/diry/dirz`（度）、`camera.fov`で指定できます。
現在値は読み取り専用の `camera.get` から取得できます（括弧は不要です）。
`camera.get.position.x/y/z` が位置、`camera.get.rotation.x/y/z` が回転（度）、
`camera.get.fov` がFOV（度）です。設定を変更すると参照値にも直ちに反映されます。

```cpp
double x = camera.get.position.x;
Vec3 rotation = camera.get.rotation;
double fov = camera.get.fov;
```

CLIで指定した`width`、`height`、`sample`、`framerate`は読み取り専用変数です。

`u_timer`と`u_resolution`も読み取り専用で参照できます。

- `u_timer`（double）: 現在のフレーム時刻（秒）。最初は`0`で、`object.render()`が成功するたびに`1 / framerate`進みます。実際の処理時間には依存しません。
- `u_resolution.x` / `u_resolution.y`（double）: 出力の幅 / 高さ（ピクセル）。CLIの`-w` / `-h`を反映します。

```cpp
double aspect = u_resolution.x / u_resolution.y;
double angle = u_timer * 90;
```

### 動画

```cpp
void example = object.init("assets/model.obj");
for (int frame = 0; frame < framerate * 2; ++frame) {
    object.push(example, {0, 0, 3}, {0, u_timer * 90, 0}, {1,1,1},
                {1,1,1}, {0.2,0.2,0.2});
    object.render();
}
```

`raymotion export out.mp4 -t 5`（または`--time 5`）で最大5秒の動画を出力します。
省略時は最大10秒です。フレーム数は`time × framerate`の小数部分を切り捨て、
1フレーム未満の指定はエラーになります。上限に達した`object.render()`でプログラムは正常終了し、
以降のコードは実行されません。MP4では`main.ray`全体を上限まで繰り返し実行するため、
`object.render()`を1回書くだけで動画を生成できます。各実行で時刻に応じた計算をやり直し、
モデルの読み込みキャッシュとカメラは保持します。描画のない繰り返しはエラーになります。
PNGでは`main.ray`を1回だけ実行し、時間制限は適用しません。これは動画の長さの上限であり、処理の実行時間制限ではありません。

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
