# generate_database.py
import os
import shutil
from xfoil_wrapper.core import generate_polar_data
import time

# --- 設定 ---
# 1. 実行したい翼型 (airfoil_data/dat_files/ に .dat が必要)
AIRFOILS_TO_RUN = [
    "aquilla",
    "clarky",
    "dae11",
    "dae21",
    "dae31",
    "e61",
    "geminism",
    "goe795",
    "mh32",
    "naca4412",
    "naca6409",
    "s1223",
    "s8035"
]

# 2. 実行したいレイノルズ数
REYNOLDS_LIST = [10000, 15000, 20000, 30000, 50000, 75000]

# 3. 迎角の範囲
AOA_START = -5.0
AOA_END = 15.0
AOA_STEP = 0.5

# 4. 入出力ディレクトリ
DAT_DIR = "airfoil_data/dat_files"
CSV_DIR = "airfoil_data/csv_polars"

# --- 実行 ---
if __name__ == "__main__":
    print("--- 🛠️  Step 4: Building Airfoil Database ---")
    
    # 出力先ディレクトリがなければ作成
    if not os.path.exists(CSV_DIR):
        os.makedirs(CSV_DIR)

    total_start_time = time.time()
    
    for airfoil_name in AIRFOILS_TO_RUN:
        
        dat_file = os.path.join(DAT_DIR, f"{airfoil_name}.dat")
        if not os.path.exists(dat_file):
            print(f"Warning: {dat_file} not found. Skipping {airfoil_name}.")
            continue
            
        print(f"\nProcessing Airfoil: {airfoil_name}")
        
        for re in REYNOLDS_LIST:
            print(f"  Calculating for Re = {re}...")
            
            # 出力ファイルパス (例: airfoil_data/csv_polars/s1223_re_10000.csv)
            output_filename = f"{airfoil_name}_re_{re}.csv"
            output_path = os.path.join(CSV_DIR, output_filename)
            
            # XFOILを呼び出してバッチ処理を実行
            success = generate_polar_data(
                airfoil_name=airfoil_name,
                dat_file_path=dat_file,
                reynolds=re,
                output_csv_path=output_path,
                aoa_start=AOA_START,
                aoa_end=AOA_END,
                aoa_step=AOA_STEP
            )
            
            if success:
                print(f"  -> Saved to {output_path}")
            else:
                print(f"  -> FAILED for Re = {re}")

    total_time = time.time() - total_start_time
    print("\n------------------------------------------")
    print(f"✅ Database generation complete in {total_time:.2f} seconds.")
