# generate_database.py
import os
import shutil
from xfoil_wrapper.core import generate_polar_data
import time
import glob # ◀ インポート追加

# --- 設定 ---
# 1. 実行したい翼型 (airfoil_data/dat_files/ から自動取得)
DAT_DIR = "airfoil_data/dat_files"
CSV_DIR = "airfoil_data/csv_polars"

# 🔽 [修正] dat_files ディレクトリから自動で翼型リストを生成 🔽
dat_files = glob.glob(os.path.join(DAT_DIR, "*.dat"))
AIRFOILS_TO_RUN = [
    os.path.basename(f).replace(".dat", "").lower() for f in dat_files
]
# 🔼 [修正完了] 🔼


# 2. 実行したいレイノルズ数
REYNOLDS_LIST = [10000, 15000, 20000, 30000, 50000, 75000]

# 3. 迎角の範囲
AOA_START = -5.0
AOA_END = 15.0
AOA_STEP = 0.5

# 4. 入出力ディレクトリ (上で定義済み)


# --- 実行 ---
if __name__ == "__main__":
    print("--- 🛠️  Step 4: Building Airfoil Database ---")
    
    # [修正] 見つかった翼型を表示
    if not AIRFOILS_TO_RUN:
        print(f"Error: No .dat files found in {DAT_DIR}. Please add airfoil files.")
    else:
        print(f"Found {len(AIRFOILS_TO_RUN)} airfoils in {DAT_DIR}:")
        print(f"  {AIRFOILS_TO_RUN}")
    
    # 出力先ディレクトリがなければ作成
    if not os.path.exists(CSV_DIR):
        os.makedirs(CSV_DIR)

    total_start_time = time.time()
    
    for airfoil_name in AIRFOILS_TO_RUN:
        
        # [修正] datファイルパスを小文字のリストから再構築
        dat_file = os.path.join(DAT_DIR, f"{airfoil_name}.dat")
        if not os.path.exists(dat_file):
             # 大文字/小文字の不一致などで見つからない場合
             print(f"Warning: {dat_file} not found (check case sensitivity?). Skipping {airfoil_name}.")
             continue
            
        print(f"\nProcessing Airfoil: {airfoil_name}")
        
        for re in REYNOLDS_LIST:
            print(f"  Calculating for Re = {re}...")
            
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
