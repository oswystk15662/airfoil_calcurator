# ドローン用プロペラ最適化フレームワーク

これは、Pythonで構築されたドローン用プロペラ（ダクト付き・ダクトなし対応）の空力設計および最適化ツールです。

特定の動作条件（回転数、消費電力）において、推力を最大化するプロペラの形状（ピッチ分布、弦長分布、翼型）を自動で探索します。

## 🏛️ システム概要

このシステムは、`OptDuct`論文（JAXA-RM-21-006）と翼素運動量理論（BEMT）に基づいています。

1.  **データベース生成 (XFOIL Automation)**
    指定された翼型（`.dat`ファイル）とレイノルズ数（Re）のリストに基づき、`xfoil.exe` を自動でバッチ処理します。これにより、各翼型の空力性能（CL/CDポーラーカーブ）がCSVファイルとして `airfoil_data/csv_polars/` に保存されます。

2.  **性能解析 (BEMT Solver)**
    `bemt_solver` モジュールが、プロペラの形状定義（ピッチ、弦長、翼型）と動作条件（RPM、対気速度）を入力として受け取ります。
    `airfoil_database_airfoiltools.py` が（1）で生成されたCSVを読み込み、Re数と迎角（AoA）に応じたCL/CDを2D補間してソルバーに提供します。
    `duct.py` モジュールが `OptDuct` 論文の理論に基づき、ダクトの有無による後流収縮と推力への寄与（リップファクター）を計算します。
    これらを組み合わせて、プロペラ全体の総推力と吸収パワーを算出します。

3.  **形状最適化 (Optuna)**
    `main_optimizer.py` が `Optuna` を使用し、（2）の性能解析関数を目的関数として呼び出します。
    「吸収パワーを指定値以下（例: 3.26W）に抑えつつ、ホバー推力を最大化する」という制約付き最適化を実行し、最適な「ピッチ分布」「弦長分布」「翼型（根本、中間、翼端）」の組み合わせを発見します。

## 📁 ディレクトリ構成

```
drone_optimizer/
│
├── airfoil_data/
│   ├── dat_files/                # [入力] 翼型の.datファイルをここに入れる
│   │   ├── s1223.dat
│   │   ├── e61.dat
│   │   └── ...
│   └── csv_polars/               # [自動生成] XFOILの計算結果(CSV)が保存される
│       ├── s1223_re_10000.csv
│       └── ...
│
├── bemt_solver/                  # BEMTソルバーのコアロジック
│   ├── __init__.py
│   ├── core.py                   # BEMTのメイン計算（fsolveによる誘導速度計算）
│   ├── duct.py                   # OptDuct理論に基づく後流収縮・ダクト推力計算
│   ├── geometry.py               # Propellerクラス（形状定義）
│   └── losses.py                 # プラントルの翼端・翼根損失
│
├── xfoil_wrapper/                # XFOIL自動化モジュール
│   ├── __init__.py
│   ├── core.py                   # XFOILをsubprocessで実行する関数 (generate_polar_data)
│   └── utils.py                  # XFOILの出力ファイル(.pol)をCSVに整形
│
├── airfoil_database_airfoiltools.py  # csv_polars/ からデータを読み込み、2D補間する
├── generate_database.py          # [実行1] XFOILをバッチ実行し、データベースを生成
├── main_optimizer.py             # [実行2] Optunaを使って最適化を実行
│
├── xfoil.exe                     # [必須] XFOIL v6.99 実行ファイル
└── (uv.exe / requirements.txt)   # Python環境管理
```

## ⚙️ 依存ライブラリ (Setup)

このプロジェクトは Python 3.10 以降を推奨します。
必要なライブラリは以下の通りです。

  * `numpy`
  * `scipy` (BEMTソルバー (fsolve) と 2D補間 (RegularGridInterpolator) で使用)
  * `pandas` (CSVデータベースの読み書きで使用)
  * `optuna` (最適化エンジン)

`uv` (または `pip`) を使ってインストールできます:

```bash
uv pip install numpy scipy pandas optuna
```

**【重要】外部プログラム:**
Windows版の `xfoil.exe` (v6.99推奨) をダウンロードし、プロジェクトのルートディレクトリ（`main_optimizer.py` と同じ場所）に配置する必要があります。

## 🚀 使い方 (Workflow)

このシステムは2つのステップで実行します。

### ステップ 1: 空力データベースの生成

まず、最適化に使用する翼型の性能データをXFOILで計算し、ローカルに保存します。

1.  **`.dat` ファイルの準備:**
    使用したい翼型の座標ファイル (`.dat` 形式) を `airfoil_data/dat_files/` フォルダに配置します。
    (例: `s1223.dat`, `e61.dat`, `naca4412.dat` ...)

2.  **`generate_database.py` の編集:**

      * `AIRFOILS_TO_RUN` リストに、`dat_files` に配置したファイル名（拡張子なし、小文字）を登録します。
      * `REYNOLDS_LIST` に、計算したいレイノルズ数のリストを指定します（例: `[10000, 15000, 20000, 30000, 50000]`）。

3.  **データベース生成の実行:**
    ターミナルで以下のコマンドを実行します。

    ```bash
    uv run .\generate_database.py
    ```

    XFOILが（非表示で）起動し、`airfoil_data/csv_polars/` に `s1223_re_10000.csv` のようなファイルが順次生成されます。
    *（注: XFOILの計算が収束しなかった場合、`FAILED` と表示され、そのCSVは生成されません。これは正常な動作です。）*

### ステップ 2: プロペラ形状の最適化

データベースが完成したら、`Optuna` を使って最適化を実行します。

1.  **`main_optimizer.py` の編集:**

      * **`[設計の基本パラメータ]`**: `DIAMETER`, `NUM_BLADES`, `RPM` などを、設計したいドローンの仕様（Telloのスペックなど）に合わせます。
      * **`[最適化の制約]`**: `TARGET_POWER_LIMIT` (最大許容パワー) や `TARGET_THRUST_MIN` (最低限必要な推力) を設定します。
      * **`[最適化の探索空間]`**:
          * `R_COORDS`: 形状を定義する半径位置（例: 3点、5点）を決めます。
          * `AIRFOIL_CHOICES`: ステップ1でデータベースに**正常に登録された**翼型名をリストにします。
          * `evaluate_design` 関数内の `trial.suggest_...` で、ピッチ角や弦長の探索範囲（上限・下限）を調整します。
      * **ダクトの設定 (重要):**
          * ダクトなし (Telloなど) の場合: `evaluate_design` 内の `Propeller` オブジェクト作成時に `duct_length=0.0` のままにします。
          * ダクトありの場合: `duct_length` と `duct_lip_radius` に値を設定します。（これらも `trial.suggest_float` を使って最適化変数に加えることも可能です）

2.  **最適化の実行:**
    ターミナルで以下のコマンドを実行します。

    ```bash
    uv run .\main_optimizer.py
    ```

    `Optuna` が（`n_trials` で指定した回数）BEMTソルバーを実行し、制約を満たしながら推力（`Best Thrust`）を最大化するパラメータの組み合わせを探索します。

3.  **結果の確認:**
    計算が完了すると、コンソールに `✅ Best solution found:` が表示され、最も性能が良かった設計の `Optimal Parameters`（翼型、弦長、ピッチ角）が出力されます。