# generate_database.py (並列化・高速化版)
import os
import glob
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing
from tqdm import tqdm # 進捗バー用

# 既存の関数をインポート
from xfoil_wrapper.core import generate_polar_data

# --- 設定 ---
DAT_DIR = "airfoil_data/dat_files"
CSV_DIR = "airfoil_data/csv_polars"

# 計算条件
REYNOLDS_LIST = [10000, 15000, 20000, 30000, 50000, 75000]
AOA_START = -5.0
AOA_END = 15.0
AOA_STEP = 0.5

# --- ヘルパー関数: 1つのタスクを実行するラッパー ---
def process_single_case(args):
    """
    並列処理ワーカーから呼ばれる関数。
    引数をタプルで受け取り、generate_polar_data を実行する。
    """
    airfoil_name, dat_file_path, re = args
    
    output_filename = f"{airfoil_name}_re_{re}.csv"
    output_path = os.path.join(CSV_DIR, output_filename)
    
    # すでに成功したCSVがある場合はスキップするロジックを入れても良いが、
    # ここでは「再生成」を優先して常に実行する
    
    success = generate_polar_data(
        airfoil_name=airfoil_name,
        dat_file_path=dat_file_path,
        reynolds=re,
        output_csv_path=output_path,
        aoa_start=AOA_START,
        aoa_end=AOA_END,
        aoa_step=AOA_STEP
    )
    
    return airfoil_name, re, success

# --- メイン処理 ---
if __name__ == "__main__":
    print("--- 🛠️  Step 4: Building Airfoil Database (Parallelized) ---")
    
    # 1. 翼型リストの作成
    dat_files = glob.glob(os.path.join(DAT_DIR, "*.dat"))
    airfoils_to_run = [
        os.path.basename(f).replace(".dat", "").lower() for f in dat_files
    ]
    
    if not airfoils_to_run:
        print(f"Error: No .dat files found in {DAT_DIR}.")
        exit()
    else:
        print(f"Found {len(airfoils_to_run)} airfoils.")

    # 出力先作成
    if not os.path.exists(CSV_DIR):
        os.makedirs(CSV_DIR)

    # 2. タスク（仕事）のリストを作成
    # (翼型, datパス, Re数) の組み合わせを全部作る
    tasks = []
    for airfoil_name in airfoils_to_run:
        dat_file = os.path.join(DAT_DIR, f"{airfoil_name}.dat")
        if not os.path.exists(dat_file):
            continue
            
        for re in REYNOLDS_LIST:
            tasks.append((airfoil_name, dat_file, re))

    # CPUコア数の取得 (論理コア数)
    max_workers = multiprocessing.cpu_count()
    print(f"Starting parallel execution with {max_workers} workers...")
    print(f"Total tasks: {len(tasks)}")
    
    total_start_time = time.time()
    
    # 3. 並列実行
    success_count = 0
    fail_count = 0
    
    # ProcessPoolExecutorで並列化
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        # tqdmで進捗バーを表示
        # executor.submit でタスクを投げ、as_completed で終わった順に処理
        futures = [executor.submit(process_single_case, task) for task in tasks]
        
        for future in tqdm(as_completed(futures), total=len(tasks), unit="polars"):
            airfoil, re, is_success = future.result()
            if is_success:
                success_count += 1
            else:
                fail_count += 1
                # 失敗したときだけ詳細を表示したい場合はコメントアウトを外す
                # print(f"\nFailed: {airfoil} @ Re={re}")

    total_time = time.time() - total_start_time
    
    print("\n------------------------------------------")
    print(f"✅ Database generation complete in {total_time:.2f} seconds.")
    print(f"   Success: {success_count}")
    print(f"   Failed:  {fail_count}")
    print("------------------------------------------")