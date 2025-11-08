# ドローン用プロペラ最適化フレームワーク (BEMT + Optuna)

これは、Pythonで構築されたドローン用プロペラの空力設計および最適化ツールです。

`Optuna` を使用して、指定された動作条件（回転数、消費電力制約）において、ホバー推力を最大化するプロペラの形状（ブレード数、ハブ比率、ピッチ/弦長分布、翼型）を自動で探索します。

設計はBézier曲線を用いてパラメータ化されており、CAD化に適した滑らかな形状を出力します。

## 🏛️ システム概要

このシステムは、翼素運動量理論（BEMT）を中核とし、XFOILによる空力データベースとOptunaによる最適化を組み合わせています。

1.  **データベース生成 (XFOIL Automation)**
    `generate_database.py` を実行すると、`airfoil_data/dat_files/` 内の全翼型（`.dat`）に対し、指定されたレイノルズ数（Re）リストに基づき `xfoil.exe` が自動でバッチ処理されます。各翼型の性能（CL/CDポーラーカーブ）が `airfoil_data/csv_polars/` にCSVとして保存されます。

2.  **性能解析 (BEMT Solver)**
    `bemt_solver` モジュールが、プロペラの形状定義と動作条件（RPM、対気速度）を入力として受け取ります。`airfoil_database_airfoiltools.py` が（1）で生成されたCSVを読み込み、Re数と迎角（AoA）に応じたCL/CDを**2D補間**してソルバーに提供します。

3.  **形状最適化 (Optuna)**
    `main_optimizer.py` が `Optuna` を使用し、（2）の性能解析関数を目的関数として呼び出します。設計変数は以下の通りです：

      * **グローバル変数**: ブレード枚数、ハブ比率。
      * **分布変数 (Bézier制御点)**: ピッチ角分布（5点）、弦長分布（5点）。
      * **カテゴリ変数**: 翼型分布（根本・中間・先端の3点）。

    「吸収パワーを指定値以下に抑えつつ、ホバー推力を最大化する」という制約付き最適化を実行し、最適なパラメータ群を発見します。

### ダクト形状について

**本システムは現在、ダクトなしプロペラ（オープンプロペラ）の最適化を行うよう設定されています。**

  * [cite\_start]`bemt_solver`（`core.py` および `duct.py`）は、`OptDuct` 論文 [cite: 496-560] に基づくダクト推力（リップファクター）の計算ロジックを**すでに搭載しています**。
  * しかし、`main_optimizer.py` 内では、Telloの仕様に合わせて `duct_length=0.0` と固定されています。
  * 将来的にダクト形状も最適化する場合は、`main_optimizer.py` の `evaluate_design` 関数内で `duct_length` と `duct_lip_radius` を `trial.suggest_float` を使って変数化することで、容易に拡張が可能です。

## 📁 ディレクトリ構成

```
drone_optimizer/
│
├── airfoil_data/
│   ├── dat_files/                # [入力] 翼型の.datファイルをここに入れる
│   │   └── s1223.dat
│   └── csv_polars/               # [自動生成] XFOILの計算結果(CSV)が保存される
│       └── s1223_re_10000.csv
│
├── bemt_solver/                  # BEMTソルバーのコアロジック
│   ├── __init__.py
│   ├── core.py                   # BEMTのメイン計算（fsolveによる誘導速度計算）
│   ├── duct.py                   # OptDuct理論に基づくダクト推力計算
│   ├── geometry.py               # Propellerクラス（形状定義）
│   └── losses.py                 # プラントルの翼端・翼根損失
│
├── xfoil_wrapper/                # XFOIL自動化モジュール
│   ├── __init__.py
│   ├── core.py                   # XFOILをsubprocessで実行する関数 (generate_polar_data)
│   └── utils.py                  # XFOILの出力ファイル(.pol)をCSVに整形
│
├── airfoil_database_airfoiltools.py  # CSVデータを読み込み、2D補間する
├── generate_database.py          # [実行1] データベースを生成するスクリプト
├── main_optimizer.py             # [実行2] Optunaで最適化を実行するメインスクリプト
│
├── xfoil.exe                     # [必須] XFOIL v6.99 実行ファイル
└── (uv.exe / requirements.txt)   # Python環境管理
```

## ⚙️ 依存ライブラリ (Setup)

このプロジェクトは Python 3.10 以降を推奨します。
必要なライブラリは以下の通りです。

  * `numpy`
  * `scipy` (BEMTソルバー (fsolve)、2D補間、Bézier曲線 (comb) で使用)
  * `pandas` (CSVデータベースの読み書きで使用)
  * `optuna` (最適化エンジン)

`uv` (または `pip`) を使ってインストールできます:

```bash
uv add numpy scipy pandas optuna
```

**【重要】外部プログラム:**
Windows版の `xfoil.exe` (v6.99推奨) をダウンロードし、プロジェクトのルートディレクトリ（`main_optimizer.py` と同じ場所）に配置する必要があります。
cloneすればそのまま入ってますが、何らかの理由で動かなかったら自分で入れてください

## 🚀 使い方 (Workflow)

このシステムは2つのステップで実行します。

### ステップ 1: 空力データベースの生成

まず、最適化に使用する翼型の性能データをXFOILで計算します。

1.  **`.dat` ファイルの準備:**
    使用したい翼型の座標ファイル (`.dat` 形式) を `airfoil_data/dat_files/` フォルダに配置します。

2.  **`generate_database.py` の編集:**

      * `REYNOLDS_LIST`: 計算したいレイノルズ数のリストを指定します（例: `[10000, 15000, 20000, 30000, 50000]`）。
      * `AOA_START`, `AOA_END`, `AOA_STEP`: 計算する迎角の範囲を指定します。

3.  **データベース生成の実行:**
    ターミナルで以下のコマンドを実行します。

    ```bash
    uv run .\generate_database.py
    ```

    `dat_files` フォルダ内の翼型が自動的に検出され、`airfoil_data/csv_polars/` に性能データCSVが生成されます。（XFOILが低Reで収束に失敗したファイルは `FAILED` と表示され、スキップされます）

### ステップ 2: プロペラ形状の最適化

データベースが完成したら、`Optuna` を使って最適化を実行します。

1.  **`main_optimizer.py` の編集:**

      * **`[設計の基本パラメータ]`**: `DIAMETER`, `RPM` などを、設計したいドローンの仕様に合わせます。
      * **`[計算精度と制御点]`**: `NUM_BEMT_ELEMENTS` (BEMTの計算分割数、例: 20)、`NUM_GEOM_CONTROL_POINTS` (形状制御点の数、例: 5) を設定します。
      * **`[最適化の制約]`**: `TARGET_POWER_LIMIT` (最大許容パワー, W)、`TARGET_THRUST_MIN` (最低推力, N)、`MIN_HUB_RADIUS_M` (物理的なハブ半径, m) を設定します。
      * **`evaluate_design` 関数内**: `trial.suggest_...` の行で、各変数（ブレード数、ハブ比率、弦長、ピッチ角）の\*\*探索範囲（上限・下限）\*\*を調整します。

2.  **最適化の実行:**
    ターミナルで以下のコマンドを実行します。

    ```bash
    uv run .\main_optimizer.py
    ```

    `Optuna` が（`n_trials` で指定した回数）BEMTソルバーを実行し、制約を満たしながらホバー推力を最大化するパラメータの組み合わせを探索します。

3.  **結果の確認:**
    計算が完了すると、コンソールに `✅ Best solution found:` が表示され、最も性能が良かった設計の `Optimal Parameters`（制御点など）が出力されます。
    同時に、`result_MMDDHHMM.txt` という名前のテキストファイルが生成され、この実行結果が保存されます。

### ステップ 3: CADへの落とし込み

出力された結果ファイル (`result_....txt`) の末尾にある
`--- CAD Data (BEMT Points Definition) ---` セクションを参照します。

このテーブルには、Bézier曲線で滑らかに補間された20点の定義（半径、ピッチ角、弦長）が記載されています。
これを使い、SolidWorksなどのCADソフトウェアで以下の手順でモデリングします。

1.  **スタッキング軸の決定:** 一般的に、翼型の **1/4弦長点（c/4）** を通る半径方向の直線をねじり軸（スタッキング軸）とします。
2.  **プロファイルの配置:**
      * `Radius (m)` の位置にスケッチ平面を作成します。
      * `Nearest Airfoil` で指定された翼型を配置します。
      * 翼型の1/4弦長点をスタッキング軸に合わせます。
      * 弦長が `Chord (mm)` と一致するようにスケーリングします。
      * スタッキング軸を中心に `Pitch (deg)` だけ回転（ねじり）させます。
3.  **ロフト:** 20個のスケッチプロファイルをロフト機能で繋ぎ、ブレード形状を生成します。
4.  **仕上げ:** ブレード枚数（`num_blades`）に合わせて円形パターンで複製し、ハブ（`hub_ratio`）と結合します。