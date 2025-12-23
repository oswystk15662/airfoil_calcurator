# ドローン用プロペラ最適化フレームワーク (BEMT + Optuna + SolidWorks Automation)

これは、PythonとC#で構築されたドローン用プロペラの空力設計・最適化およびCAD自動生成ツールです。

`Optuna` を使用して、指定された動作条件（回転数、消費電力制約）においてホバー推力を最大化するプロペラ形状を自動探索し、連携するC#マクロを用いてSolidWorks上で3Dモデルを自動生成します。

## 🏛️ システム概要

このシステムは、翼素運動量理論（BEMT）を中核とし、XFOILによる空力データベース、Optunaによる最適化、そしてSolidWorks APIによるモデリングを組み合わせています。
[参考：JAXA OptDuct](https://jaxa.repo.nii.ac.jp/record/48266/files/AA2130014000.pdf) MATLABにあるらしいが、発見できず、Google Geminiにこのpdfを食わせて作りました、実装によって誤差が大きくなってたらすみません。また、DJI Telloという翼端でRe20000ぐらいしか出ないドローンを前提としているので、別の用途で使いたかったらご自身で編集してください。

1.  **データベース生成 (XFOIL Automation)**
    `generate_database.py` を実行すると、`airfoil_data/dat_files/` 内の全翼型（`.dat`）に対し、指定されたレイノルズ数（Re）リストに基づき `xfoil.exe` が自動でバッチ処理されます。各翼型の性能（CL/CDポーラーカーブ）が `airfoil_data/csv_polars/` にCSVとして保存されます。

2.  **性能解析 (BEMT Solver)**
    `bemt_solver` モジュールが、プロペラの形状定義と動作条件（RPM、対気速度）を入力として受け取ります。`airfoil_database_airfoiltools.py` が（1）で生成されたCSVを読み込み、Re数と迎角（AoA）に応じたCL/CDを**2D補間**してソルバーに提供します。

3.  **形状最適化 (Optuna)**
    `main_optimizer.py` が `Optuna` を使用し、（2）の性能解析関数を目的関数として呼び出します。
    「吸収パワーを指定値以下に抑えつつ、ホバー推力を最大化する」という制約付き最適化を実行し、最適なパラメータ群を発見します。
    * **検証実績**: 最適化されたプロペラは、過度なトルク変動を起こさず、一般的なESC/FCでの制御が可能です。

4.  **CAD自動生成 (C# Macro)**
    最適化結果を読み込み、SolidWorks APIを介して起動済みのSolidworks2025 sp0.5と通信し、新しい部品として、ブレードの断面生成、ねじり下げ（Twist）、ロフト、ハブ結合までを完全自動で行います。

### ダクト形状について

~~本システムは現在、ダクトなしプロペラ（オープンプロペラ）の最適化および生成を主としています。~~
ダクト込みで最適化していますが、ダクトありで実機検証したわけではないのでなんとも言えません。（DJI TelloのCADがなくて、、、）

  * [cite_start]`bemt_solver`（`core.py` および `duct.py`）は、`OptDuct` 論文 に基づくダクト推力計算ロジックを搭載しています。
  * `main_optimizer.py` 内でダクト長などの変数を有効化することで、ダクト付きプロペラの設計も可能です。
  * 実機検証においては、ダクトなしのオープンプロペラ構成で十分な静音性と推力効率が得られることが確認されています。

## 📁 ディレクトリ構成


```

drone_optimizer/
│
├── airfoil_data/
│   ├── dat_files/                # [手動入力] 翼型の.datファイルをここに入れる
│   └── csv_polars/               # [自動生成] XFOILの計算結果(CSV)
│
├── bemt_solver/                  # BEMTソルバーのコアロジック
│   ├── core.py                   # BEMTメイン計算
│   ├── duct.py                   # OptDuct理論
│   ├── geometry.py               # Propellerクラス
│   └── losses.py                 # プラントル損失係数
│
├── PropellerLofter/              # [C#] SolidWorks自動モデリング用プロジェクト
│   ├── PropellerLofter.exe       # これをダブルクリックで実行できます
│   └── ~~                        # 依存、solidworks2025、及びAPIが入ってることを前提とします。
│
├── xfoil_wrapper/                # generate_databaseで使うxfoilのwrapper
│   ├── core.py                   # メインの呼び出しラッパー
│   └── utils.py                  # xfoilが出すデータを使いやすいようにしたり、
│
├── airfoil_database_airfoiltools.py
├── export_3d_curves.py           # [Python] 最適化結果を3D点群データに変換・リサンプリングする
├── generate_database.py          # [Step 1] データベース生成
├── main_optimizer.py             # [Step 2] 最適化実行
│
├── xfoil.exe                     # [必須] XFOIL v6.99
└── (uv.exe / requirements.txt)   # Python環境管理

```

## ⚙️ 依存ライブラリ (Setup)

### Python環境
Python 3.10 以降を推奨。
```bash
uv add numpy scipy pandas optuna

```

**外部プログラム:** Windows版 `xfoil.exe` (v6.99) をルートディレクトリに配置してください。cloneすればそのまま入ります（作法がなってないのは許してください、ignoreし忘れてまぁいっかしました）

### CAD環境

* **SolidWorks**: 2025以降（推奨）
~~* **Visual Studio**: 2022以降（C#開発環境、.NET 4.8が必要）~~ exe配布なので多分入れなくてよい

## 🚀 使い方 (Workflow)

### ステップ 1: 空力データベースの生成

翼型データ (`.dat`) を配置し、データベースを構築します。

```bash
uv run .\generate_database.py

```

### ステップ 2: プロペラ形状の最適化

Optunaを実行し、最適な設計パラメータを探索します。
Telloクラスなどの小型機の場合、ハブ径の制約などを `main_optimizer.py` で調整してください。

```bash
uv run .\main_optimizer.py

```

計算が完了すると、`result_MMDDHHMM.txt` に結果が出力されます。

### ステップ 3: 3D点群データのリサンプリング

最適化結果の翼型座標を、CADで扱いやすい等間隔な点群データに変換します。

```bash
uv run .\export_3d_curves.py

```

`3d_curves_output/` フォルダに断面データが生成されます。

### ステップ 4: SolidWorksでの自動生成

1. SolidWorksを起動します。
2. PropellerLofter.exeをダブルクリックします
3. 表示されたウィンドウでステップ3で出力されたフォルダを選択します。（もしウィンドウが表示されなかったら、一番上に来なかっただけなので、タスクバーのデスクトップ切り替えアイコンをクリックし、フォルダを選べそうなウィンドウを選択して下さい。）
4. **自動実行**: ハブの生成、各断面のスケッチ配置、ガイドカーブを考慮したロフト、円形パターンなどの処理が自動で行われ、3Dモデルが完成します。

完成したモデルは、必要に応じてフィレット追加などの仕上げを行い、3Dプリントしてください。
