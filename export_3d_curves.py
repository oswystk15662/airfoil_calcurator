import os
import glob
import numpy as np
import math
from scipy.interpolate import interp1d

# --- 設定 ---
DAT_DIR = "airfoil_data/dat_files"
OUTPUT_ROOT = "3d_curves_output"
# [新規設定] すべての翼型をこの点数に揃える (多すぎると重くなるので100-150推奨)
RESAMPLE_POINTS = 100 

def find_latest_result_file():
    search_path = os.path.join("optimization_results", "result_*.txt")
    files = glob.glob(search_path)
    if not files:
        files = glob.glob("result_*.txt")
    if not files:
        return None
    files.sort(key=os.path.getmtime, reverse=True)
    return files[0]

def parse_result_file(filepath):
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
        if "Radius" in line or "--|--" in line:
            continue
            
        parts = line.split('|')
        if len(parts) < 5:
            continue
        try:
            idx = int(parts[0].strip())
            radius_m = float(parts[1].strip())
            pitch_deg = float(parts[2].strip())
            chord_mm = float(parts[3].strip())
            airfoil = parts[4].strip()
            
            sections.append({
                "index": idx,
                "radius_mm": radius_m * 1000.0,
                "pitch_deg": pitch_deg,
                "chord_mm": chord_mm,
                "airfoil": airfoil
            })
        except ValueError:
            continue
    return sections

def read_dat_file(airfoil_name):
    """ .dat ファイルを読み込んで (x, y) 座標のリストを返す """
    search_pattern = os.path.join(DAT_DIR, f"{airfoil_name}.dat")
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
        for line in lines:
            parts = line.strip().split()
            if len(parts) >= 2:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    if x > 10.0 or x < -10.0: continue
                    coords.append([x, y])
                except ValueError:
                    continue
    return np.array(coords)

def resample_curve(coords, num_points):
    """
    点群を曲線に沿って等間隔にリサンプリングし、指定された点数にする。
    これにより、全ての翼型の頂点数が一致する。
    """
    # 座標が空ならそのまま返す
    if len(coords) < 2:
        return coords

    # 重複点の削除 (距離が0の連続点を消す)
    # diff = np.diff(coords, axis=0)
    # dist = np.linalg.norm(diff, axis=1)
    # mask = np.concatenate(([True], dist > 1e-9))
    # coords = coords[mask]

    x = coords[:, 0]
    y = coords[:, 1]

    # 曲線に沿った累積距離を計算
    # (0, d1, d1+d2, ...)
    dist = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    cum_dist = np.concatenate(([0], np.cumsum(dist)))
    
    total_length = cum_dist[-1]
    
    # 距離に基づいた補間関数を作成
    # linear補間で十分滑らか (datファイル自体が密なので)
    fx = interp1d(cum_dist, x, kind='linear')
    fy = interp1d(cum_dist, y, kind='linear')
    
    # 新しい等間隔の距離点を作成
    new_dist = np.linspace(0, total_length, num_points)
    
    # 新しい座標を計算
    new_x = fx(new_dist)
    new_y = fy(new_dist)
    
    return np.column_stack((new_x, new_y))

def transform_coordinates(coords, chord_mm, pitch_deg, radius_mm):
    """
    2D翼型座標を3D空間座標に変換する
    """
    transformed = []
    
    # ピッチ角の符号を反転 (時計回り = 前縁持ち上げ)
    theta = math.radians(-pitch_deg) 
    
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    
    stacking_offset_x = 0.25
    
    for x_raw, y_raw in coords:
        # 1. センタリング
        x_centered = x_raw - stacking_offset_x
        y_centered = y_raw
        
        # 2. スケーリング
        x_scaled = x_centered * chord_mm
        y_scaled = y_centered * chord_mm
        
        # 3. 回転
        x_rot = x_scaled * cos_t - y_scaled * sin_t
        y_rot = x_scaled * sin_t + y_scaled * cos_t
        
        # 4. 配置
        transformed.append((x_rot, y_rot, radius_mm))
        
    return transformed

def main():
    print("--- 🛠️  Exporting 3D Curves for SolidWorks (Resampled) ---")
    
    # result_file = find_latest_result_file()
    result_file = "C:\\Users\\oswys\\Documents\\sd_technology_ensyu\\airfoil_calcurator\\optimization_results\\result_12091154.txt"
    if not result_file:
        print("Error: No result_*.txt file found.")
        return
    
    print(f"Reading: {result_file}")
    sections = parse_result_file(result_file)
    
    if not sections:
        print("Error: No section data found in the file.")
        return

    timestamp = os.path.basename(result_file).replace("result_", "").replace(".txt", "")
    output_dir = os.path.join(OUTPUT_ROOT, timestamp)
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    print(f"Output Directory: {output_dir}")
    print(f"Resampling all airfoils to {RESAMPLE_POINTS} points...")
    
    for section in sections:
        idx = section['index']
        r_mm = section['radius_mm']
        airfoil = section['airfoil']
        
        coords_raw = read_dat_file(airfoil)
        if len(coords_raw) == 0:
            continue
            
        # 🔽 [追加] リサンプリング実行 🔽
        # 全ての断面が RESAMPLE_POINTS 個の頂点を持つようになる
        coords_resampled = resample_curve(coords_raw, RESAMPLE_POINTS)
        # 🔼 [追加完了] 🔼
            
        coords_3d = transform_coordinates(
            coords_resampled, 
            section['chord_mm'], 
            section['pitch_deg'], 
            r_mm
        )
        
        filename = f"section_{idx:02d}_{airfoil}_r{r_mm:.1f}.txt"
        filepath = os.path.join(output_dir, filename)
        
        with open(filepath, 'w') as f:
            for x, y, z in coords_3d:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
                
        print(f"  -> Generated: {filename}")
        
    print("\n✅ Export complete!")

if __name__ == "__main__":
    main()