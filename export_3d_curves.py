import os
import glob
import numpy as np
import math

# --- 設定 ---
# 翼型データのあるフォルダ
DAT_DIR = "airfoil_data/dat_files"
# 出力先フォルダ
OUTPUT_ROOT = "3d_curves_output"

def find_latest_result_file():
    """ optimization_results フォルダから最新の result_*.txt を探す """
    search_path = os.path.join("optimization_results", "result_*.txt")
    files = glob.glob(search_path)
    if not files:
        # ルートも探す
        files = glob.glob("result_*.txt")
        
    if not files:
        return None
    
    # 更新日時が新しい順にソート
    files.sort(key=os.path.getmtime, reverse=True)
    return files[0]

def parse_result_file(filepath):
    """ result.txt をパースして断面データを抽出する """
    sections = []
    
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        
    in_cad_section = False
    for line in lines:
        if "--- CAD Data" in line:
            in_cad_section = True
            continue
        
        if not in_cad_section:
            continue
            
        # ヘッダーや区切り線をスキップ
        if "Radius" in line or "--|--" in line:
            continue
            
        parts = line.split('|')
        if len(parts) < 5:
            continue
            
        try:
            # i | Radius (m) | Pitch (deg) | Chord (mm) | Nearest Airfoil | ...
            idx = int(parts[0].strip())
            radius_m = float(parts[1].strip())
            pitch_deg = float(parts[2].strip())
            chord_mm = float(parts[3].strip())
            airfoil = parts[4].strip()
            
            sections.append({
                "index": idx,
                "radius_mm": radius_m * 1000.0, # mmに変換
                "pitch_deg": pitch_deg,
                "chord_mm": chord_mm,
                "airfoil": airfoil
            })
        except ValueError:
            continue
            
    return sections

def read_dat_file(airfoil_name):
    """ .dat ファイルを読み込んで (x, y) 座標のリストを返す """
    # ファイル名を探す (大文字小文字を無視して検索)
    search_pattern = os.path.join(DAT_DIR, f"{airfoil_name}.dat")
    # 正確なマッチがないか glob で探す
    candidates = glob.glob(os.path.join(DAT_DIR, "*.dat"))
    
    target_file = None
    for f in candidates:
        fname = os.path.basename(f).lower()
        if fname == f"{airfoil_name.lower()}.dat":
            target_file = f
            break
            
    if not target_file:
        print(f"Warning: Airfoil file for '{airfoil_name}' not found.")
        return []

    coords = []
    with open(target_file, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()
        # 通常、1行目は名前なのでスキップ。数値が始まる行から読む
        for line in lines:
            parts = line.strip().split()
            if len(parts) >= 2:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    # まれにある異常値（タイトル行など）を除外
                    if x > 10.0 or x < -10.0: continue
                    coords.append((x, y))
                except ValueError:
                    continue
    return coords

def transform_coordinates(coords, chord_mm, pitch_deg, radius_mm):
    """
    2D翼型座標を3D空間座標に変換する
    - Scaling: 弦長倍
    - Stacking: c/4 (0.25, 0) を原点に合わせて配置
    - Rotation: ピッチ角回転
    - Translation: Z軸 = 半径
    """
    transformed = []
    
    # 回転行列の準備
    theta = math.radians(pitch_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    
    # スタッキング位置 (前縁から25%)
    stacking_offset_x = 0.25
    
    for x_raw, y_raw in coords:
        # 1. センタリング (c/4 を原点へ)
        x_centered = x_raw - stacking_offset_x
        y_centered = y_raw
        
        # 2. スケーリング (mm単位へ)
        x_scaled = x_centered * chord_mm
        y_scaled = y_centered * chord_mm
        
        # 3. 回転 (ピッチ角)
        # 通常、プロペラピッチは回転面に対して前縁が持ち上がる方向
        x_rot = x_scaled * cos_t - y_scaled * sin_t
        y_rot = x_scaled * sin_t + y_scaled * cos_t
        
        # 4. 配置
        # ユーザー要望: xy座標 + zとして半径
        # SolidWorksではテキスト読み込み時、列の順序を選べますが、
        # 一般的には X Y Z です。
        # ここでは:
        # X = 翼型のコード方向成分 (回転後)
        # Y = 翼型の厚み方向成分 (回転後)
        # Z = 半径 (Radius)
        
        transformed.append((x_rot, y_rot, radius_mm))
        
    return transformed

def main():
    print("--- 🛠️  Exporting 3D Curves for SolidWorks ---")
    
    # 1. 最新の結果ファイルを読み込み
    result_file = find_latest_result_file()
    if not result_file:
        print("Error: No result_*.txt file found.")
        return
    
    # manual カス実装ですが、動くので（）
    result_file = "C:\\Users\\oswys\\Documents\\sd_technology_ensyu\\airfoil_calcurator\\optimization_results\\result_12011408.txt"

    print(f"Reading: {result_file}")
    sections = parse_result_file(result_file)
    
    if not sections:
        print("Error: No section data found in the file.")
        return

    # 2. 出力フォルダの準備
    # 結果ファイル名に基づいたサブフォルダを作成
    timestamp = os.path.basename(result_file).replace("result_", "").replace(".txt", "")
    output_dir = os.path.join(OUTPUT_ROOT, timestamp)
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    print(f"Output Directory: {output_dir}")
    
    # 3. 各断面の処理
    for section in sections:
        idx = section['index']
        r_mm = section['radius_mm']
        airfoil = section['airfoil']
        
        # .dat読み込み
        coords_raw = read_dat_file(airfoil)
        if not coords_raw:
            continue
            
        # 座標変換
        coords_3d = transform_coordinates(
            coords_raw, 
            section['chord_mm'], 
            section['pitch_deg'], 
            r_mm
        )
        
        # ファイル書き出し (X Y Z 形式, 単位: mm)
        filename = f"section_{idx:02d}_{airfoil}_r{r_mm:.1f}.txt"
        filepath = os.path.join(output_dir, filename)
        
        with open(filepath, 'w') as f:
            for x, y, z in coords_3d:
                # SolidWorksはカンマ区切りでもスペース区切りでも読めるが、
                # 単位(mm)を明示するか、読み込み時に指定する必要がある。
                # ここでは単純な数値 (mm) を出力。
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
                
        print(f"  -> Generated: {filename}")
        
    print("\n✅ Export complete!")
    print("In SolidWorks: Insert > Curve > Curve Through XYZ Points > Browse...")
    print("Make sure to select 'Millimeters' in the import dialog.")

if __name__ == "__main__":
    main()