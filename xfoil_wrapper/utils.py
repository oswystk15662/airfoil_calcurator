# xfoil_wrapper/utils.py
import os
import re

AIRFOIL_DIR = "./xfoil_wrapper/airfoils/"

def find_airfoil_file(airfoil_name: str) -> str | None:
    """ 翼型名（例: "naca2412"）から .dat ファイルのパスを探す """
    # NACA翼型の場合は自動生成するロジックも追加可能
    
    file_path = os.path.join(AIRFOIL_DIR, f"{airfoil_name}.dat")
    if os.path.exists(file_path):
        return file_path
    return None

def generate_xfoil_input_single_aoa(filepath: str, reynolds: float, aoa: float) -> str:
    """ XFOILに渡すコマンド文字列（標準入力）を生成する（単一迎角） """
    commands = f"""
    LOAD {filepath}
    OPER
    VISC {reynolds}
    PACC
    output.pol
    
    ASEQ {aoa} {aoa} 1
    
    
    QUIT
    """
    # output.pol : 出力ファイル名（ダミー、実際は標準出力をパースする）

    # XFOILは"空行"（Enterキー）をコマンド区切りとして多用するため、
    # ヒアドキュメントの改行が重要になる
    return "\n".join([line.strip() for line in commands.splitlines()])


def parse_xfoil_output_single(stdout: str) -> (float | None, float | None, float | None):
    """ XFOILの標準出力から CL, CD, CM を正規表現で抜き出す (v6.99対応版) """
    
    # XFOIL 6.99のイテレーション出力 (a = ..., CL = ..., Cm = ..., CD = ...) を探す
    # 
    # 例:
    #   a = 5.000   CL = 0.7815
    #   Cm = -0.0519   CD = 0.01048  => ...
    
    # re.DOTALL は "." が改行文字にもマッチするようにするフラグ
    pattern = re.compile(
        r"CL =\s*([\d.-]+).*?Cm =\s*([\d.-]+).*?CD =\s*([\d.-]+)",
        re.DOTALL 
    )
    
    # findall ですべてのイテレーション結果をタプル (cl, cm, cd) のリストとして取得
    matches = pattern.findall(stdout)
    
    if matches:
        # 最後のイテレーション結果（＝収束した値）を取得
        last_match = matches[-1]
        
        try:
            cl = float(last_match[0])
            cm = float(last_match[1]) # 順番に注意！ 2番目が Cm
            cd = float(last_match[2]) # 3番目が Cd
            
            # 戻り値は (cl, cd, cm) の順番に統一して返す
            return cl, cd, cm
        except ValueError:
            # まれに "---" などがマッチした場合
            return None, None, None
    else:
        # パターンに一致するものがなかった場合
        return None, None, None