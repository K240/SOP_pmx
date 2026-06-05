# SOP_pmx

Houdini 用 の **MikuMikuDance の PMX モデル（2.0 / 2.1）** SOP ノード。

## 必要環境

- **Houdini**（対応バージョンは Houdini SDK に依存。開発・検証は 21.0.700）
- **CMake** 3.18 以上
- **C/C++ ビルド環境**（各 OS のコンパイラ）

依存ライブラリはありません（PMX 解析は同梱の `pmx_parser.{h,cpp}`）。

## ビルド方法

### 前提

- 環境変数 **`HFS`** に Houdini のインストールパスを設定すること。
- **Windows:** Visual Studio（MSVC / C++ ツール）。`cmake` は既定で Visual Studio 生成器を選択します。
- **Linux / macOS:** 各 OS の C++ コンパイラ。
- 構成（configure）時に Houdini 付属の **`hython`** が起動し、DSO の出力先（ユーザーの `dso` フォルダ）を解決します。`HFS` が正しく設定されている必要があります。

### Windows

```powershell
$env:HFS = "C:\Program Files\Side Effects Software\Houdini 21.0.700"
cmake -S . -B build
cmake --build build --config Release
```

### Linux

```bash
export HFS=/opt/hfs21.0.700
cmake -S . -B build
cmake --build build
```

### macOS

```bash
export HFS="/Applications/Houdini/Houdini21.0.700/Frameworks/Houdini.framework/Versions/Current/Resources"
cmake -S . -B build
cmake --build build
```

### 出力先（DSO）

ビルドが成功すると DSO がユーザーの **dso** フォルダに出力されます（OneDrive 等で Documents がリダイレクトされている場合はそちら）。ライブラリ名は `SOP_pmx` なのでファイル名は **`SOP_pmx`** です。

- **Windows:** `%USERPROFILE%\Documents\houdini<バージョン>\dso\SOP_pmx.dll`
- **Linux:** `~/houdini<バージョン>/dso/SOP_pmx.so`
- **macOS:** `~/Library/Preferences/houdini/<バージョン>/dso/SOP_pmx.dylib`

Houdini を起動すると SOP ネットワークに **`pmx_import`** ノードが追加されます。

### アイコン（任意）

ノードアイコンを表示するには、`SOP_pmx_import.svg` を Houdini ユーザーディレクトリの **`config/Icons/`** にコピーします（演算子のアイコン名 `SOP_pmx_import` とファイル名を一致させること）。

- **Windows:** `%USERPROFILE%\Documents\houdini<バージョン>\config\Icons\SOP_pmx_import.svg`
- **Linux:** `~/houdini<バージョン>/config/Icons/SOP_pmx_import.svg`
- **macOS:** `~/Library/Preferences/houdini/<バージョン>/config/Icons/SOP_pmx_import.svg`


## パラメータ

| パラメータ | 内容 |
|------------|------|
| **Reimport** | ファイルを再読み込み（強制 recook） |
| **pmx** (`file`) | 読み込む `.pmx` ファイル（CJK・日本語パス対応） |
| **Coordinate** | `Original`：ファイルの座標・スケールのまま。`Houdini`（既定）：左手 Y-up → 右手 Y-up（Z 反転）＋メートルへスケール |
| **Unit (meters per PMX unit)** | PMX 1 単位のメートル換算（既定 **0.08**）。`Houdini` 時のみ有効。最終スケール = `Unit ÷ HIP の単位長` |
| **Sanitize Names** | 属性・グループとして安全なトークンへ名前をエンコード（`UT_VarEncode`）。既定オフ（生の日本語名を保持）。**`pmx_material` は対象外で常に生の名前** |
| **Import Skinning** | メッシュ出力にボーンキャプチャ（スキンウェイト）を書き込む（既定オン） |

### 座標・単位変換（`Houdini` モード）

MMD は **左手系・Y-up**。`Houdini` モードでは右手系へ変換します。

- **位置**（頂点・ボーン・剛体・ジョイント・モーフデルタ）：`z → -z`、さらに `scale` 倍
- **法線**：`z → -z`
- **巻き順**：PMX は左手系で前面が時計回り。Z 反転により**同一頂点順のまま**外向き・法線整合な面になります（巻き順は反転しません）。`Original` モードでは整合のため巻き順を反転します。
- **UV**：MMD は左上原点 → `v → 1 - v`
- **回転**（剛体・ジョイント euler、ボーンモーフ quaternion）：Z 反転に伴いクォータニオンを `(x,y,z,w) → (-x,-y,z,w)` でミラー。**※ euler の合成順序（既定 YXZ）は実モデルで要検証**。生の値も `pmx_rotation` 等で保持します。

## 出力（5 アウトプット）

| # | ラベル | 内容 |
|---|--------|------|
| 0 | **Mesh** | 三角形メッシュ（法線・UV・マテリアル・ボーンキャプチャ・detail メタデータ） |
| 1 | **Skeleton** | KineFX 互換のボーン点＋親子ポリライン（IK・継承・軸など全ボーン情報） |
| 2 | **RigidBodies** | 剛体ごとの点（形状・質量・物理モード等。ソフトボディは detail カタログ） |
| 3 | **Joints** | ジョイントごとの 2 点ポリライン（拘束種別・剛体参照・制限・バネ） |
| 4 | **Morphs** | 頂点/UV モーフのデルタ点群＋モーフ/表示枠カタログ |

### 出力 0: Mesh

PMX は既にインデックス済み共有頂点のため、**PMX 頂点 i = Houdini point i**。

| 区分 | 属性 | 内容 |
|------|------|------|
| point | `P` | 位置 |
| point | `N` | 法線 |
| point | `uv` (vec3) | 基本 UV（V 反転済み）。追加 UV は `uv2`..`uv5` (各 vec4) |
| point | `pmx_edgescale` (float) | 輪郭線スケール |
| point | `pmx_sdef_c` / `pmx_sdef_r0` / `pmx_sdef_r1` (vec3) | SDEF の C/R0/R1（モデルが SDEF を含む場合のみ生成） |
| point | `boneCapture` | スキンウェイト（`addCaptureRegion`/`setCaptureWeight`。領域名＝ボーン名） |
| prim | `pmx_material` (str) | マテリアル名（ローカル優先）。**常に生の名前**（Sanitize Names の対象外） |
| prim | `material_index` (int) | マテリアル索引 |
| prim | `shop_materialpath` (str) | マテリアル名（割当用トークン。Sanitize Names を尊重） |
| prim | `pmx_texture` (str) | ディフューズテクスチャの絶対パス（CJK 相対パス解決済み） |
| detail | `pmx` (dict) | バージョン・エンコード・モデル名/コメント・各種件数・coordinate・unit/scale・ソースパス |
| detail | `pmx_materials` (dict[]) | 全マテリアルのカタログ（diffuse/specular/ambient/edge/flags/texture/sphere/toon/面数） |

**スキニング**：BDEF1=単一、BDEF2=(w,1−w)、BDEF4・QDEF=4 本（正規化）、SDEF=線形ブレンド（LBS）近似（警告を表示。C/R0/R1 は `pmx_sdef_*` に保持し、精密な SDEF 変形に利用可能）。バインドは各ボーンのワールド位置への平行移動（回転は恒等）で、出力 1 のスケルトンと整合します（Bone/Skin Deform にそのまま使用可）。

> テクスチャは PMX 参照名で解決します。再配布モデルでは参照名と実ファイル名が食い違う場合があり、その際 `pmx_texture` は解決後のパスを保持しますがファイルは存在しないことがあります。

### 出力 1: Skeleton（KineFX 互換）


| 属性 | 内容 |
|------|------|
| `name` (str) | ボーン名（ローカル優先・一意化） |
| `transform` (mat3) | 向き（既定は恒等＝バインドと整合） |
| `pmx_bone_index` / `pmx_parent` / `pmx_layer` (int) | 索引・親索引・変形階層 |
| `pmx_bone` (dict) | 全ボーン情報：フラグ群（rotatable/translatable/visible/enabled/ik/inherit_*/fixed_axis/local_axis/physics_after_deform）、tail（`tail_is_bone`/`tail_bone`/`tail_pos`）、継承（`inherit_parent`/`inherit_weight`）、固定軸・ローカル軸、外部親、IK（`ik_target`/`ik_loop`/`ik_limit_rad`/`ik_link_bones`/`ik_link_has_limit`/`ik_link_min`/`ik_link_max`） |

### 出力 2: RigidBodies


| 属性 | 内容 |
|------|------|
| `name` (str) | 剛体名（一意化。ジョイントから参照される） |
| `P` / `orient` (quat) | 位置・向き |
| `pmx_shape` (str) / `size` (vec3) / `pscale` | 形状（sphere/box/capsule）・サイズ（スケール済み） |
| `pmx_bone` (str) / `pmx_bone_index` (int) | 関連ボーン |
| `pmx_mass` / `pmx_linear_damping` / `pmx_angular_damping` / `pmx_restitution` / `pmx_friction` | 物理パラメータ |
| `pmx_group` / `pmx_noncollision_mask` / `pmx_physics_mode` (int) | 衝突グループ・非衝突マスク・物理モード（0 follow/1 physics/2 physics+bone） |
| `jolt_shapetype` / `jolt_bodytype` (int) | Jolt 整合（box=0/sphere=1/capsule=2、static=0/dynamic=1/kinematic=2。follow→kinematic、physics(+bone)→dynamic） |
| `jolt_friction` / `jolt_restitution` / `jolt_lineardamping` / `jolt_angulardamping` | Jolt 整合の物理値 |
| detail `pmx_soft_bodies` (dict[]) | ソフトボディ（PMX 2.1）カタログ：shape/material/group/flags/config(12)/cluster(6)/iteration(4)/material_coeff(3)/anchors/pins |

> Jolt の剛体は密度指定、PMX は質量指定です。質量を厳密に再現する場合は `pmx_mass` と形状体積から密度を換算してください。

### 出力 3: Joints


| 属性 | 内容 |
|------|------|
| `name` (str) | ジョイント名（一意化） |
| `pmx_joint_type` (int) / `pmx_joint_type_name` (str) | 種別（0 spring6dof, 1 6dof, 2 p2p, 3 conetwist, 4 slider, 5 hinge） |
| `jolt_constraint_type` (int) | Jolt 整合（fixed=0/point=1/hinge=2/distance=3/slider=4/cone=5/sixdof=6） |
| `jolt_body_a` / `jolt_body_b` (str) | 接続剛体名（出力 2 の `name` と一致） |
| `pmx_rigid_a` / `pmx_rigid_b` (int) | 接続剛体索引 |
| `pmx_joint_pos` / `pmx_joint_rot` | ジョイントの位置・回転 |
| `pmx_pos_limit_lower/upper` / `pmx_rot_limit_lower/upper` / `pmx_spring_pos` / `pmx_spring_rot` | 移動・回転の制限とバネ係数（位置はスケール済み、回転は PMX ジョイントローカルの生値） |

### 出力 4: Morphs

頂点・UV モーフは**デルタ点群**。

| 区分 | 属性 | 内容 |
|------|------|------|
| point | `morph_name` (str) / `morph_index` (int) / `morph_type` (int) | 所属モーフ |
| point | `vtx` (int) | 対象メッシュ point 索引（出力 0 と共通） |
| point | `delta` (vec3) | 頂点モーフのデルタ（Z 反転・スケール済み） |
| point | `uv_delta` (vec4) | UV モーフのデルタ（V 反転済み） |
| detail | `pmx_morphs` (dict[]) | 全モーフのカタログ。group/bone/material/flip/impulse はオフセットデータも内包 |
| detail | `pmx_display_frames` (dict[]) | 表示枠（bone/morph の UI グループ） |

**ブレンドシェイプ再構成例**：出力 4 から `morph_index` で目的のモーフ点を抽出し、各点の `vtx` が指す出力 0 の point に `delta`（×ウェイト）を加算します。

## 実装メモ

- PMX 文字列は UTF-16LE / UTF-8 を UTF-8 へ正規化（サロゲートペア対応）。CJK の日本語パスは `std::filesystem::u8path` で開きます。
- 参照系インデックス（bone/material/morph/rigid/texture）は符号付き（-1=なし）、頂点インデックス（面・モーフ対象）は符号無しで読み分けます。
- ボーン名・剛体名は決定的に一意化され、メッシュのキャプチャ領域名とスケルトンの点名が一致します。

## ファイル構成

```
SOP_pmx_reader.h / .C   SOP ノード（登録・パラメータ・verb cook・複数出力）
SOP_pmx_common.h        座標/単位変換・名前処理・テクスチャパス解決
pmx_parser.h / .cpp     自作 PMX パーサ（Houdini 非依存・純 C++17）
CMakeLists.txt          ビルド（dso 出力先処理込み）
```
