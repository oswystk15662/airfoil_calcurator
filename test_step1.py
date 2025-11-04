import time
from xfoil_wrapper.core import get_airfoil_performance

def run_test():
    """
    XFOILラッパーモジュールの簡単な動作確認テスト
    """
    print("--- 🚀 XFOIL Wrapper Test ---")
    
    # --- テストパラメータ ---
    # xfoil_wrapper/airfoils/naca2412.dat が存在することを前提とします
    airfoil_name = "naca2412"
    reynolds = 500000.0  # (5.0e5)
    aoa = 5.0            # 迎角 5度
    
    print(f"Testing airfoil: {airfoil_name}")
    print(f"Reynolds number: {reynolds}")
    print(f"Angle of Attack: {aoa}°")
    print("---------------------------------")
    print("Calling XFOIL...")
    
    start_time = time.time()
    
    # 実際にラッパー関数を呼び出す
    cl, cd, cm = get_airfoil_performance(airfoil_name, reynolds, aoa)
    
    end_time = time.time()
    print(f"Calculation finished in {end_time - start_time:.2f} seconds.")
    print("---------------------------------")
    
    # --- 結果の検証 ---
    if cl is not None and cd is not None:
        print("✅ Success! XFOIL successfully executed and output was parsed.")
        print(f"   Lift Coefficient (CL):   {cl:.4f}")
        print(f"   Drag Coefficient (CD):   {cd:.4f}")
        print(f"   Moment Coefficient (CM): {cm:.4f}")
        
        # NACA 2412 @ Re 500k, AoA 5° は、CL 0.8前後、CD 0.008前後になるはずです
        # (XFOILのバージョンや設定で多少変動します)
        if 0.7 < cl < 0.9 and 0.007 < cd < 0.015:
            print("   (Values seem reasonable for NACA 2412)")
        else:
            print("   (Warning: Values seem unusual, but parsing was successful)")
            
    else:
        print("❌ Failure. Could not get results from XFOIL.")
        print("Please check the following:")
        print("  1. Is 'xfoil_exec' (or 'xfoil.exe') in the root directory?")
        print("  2. Is the path in 'xfoil_wrapper/core.py' (XFOIL_EXEC_PATH) correct?")
        print("  3. Does 'xfoil_wrapper/airfoils/naca2412.dat' exist?")
        print("  4. (Linux/macOS) Is 'xfoil_exec' set as executable? (chmod +x xfoil_exec)")
        print("  5. Check the console output above for any errors from subprocess.")

    print("---------------------------------")
    print("Test complete.")

if __name__ == "__main__":
    # このスクリプトが直接実行されたときに test を実行
    run_test()