# bemt_solver/geometry.py
import numpy as np

class Propeller:
    """
    プロペラの幾何学的形状を定義するクラス。
    [修正] 形状(ピッチ/弦長)と翼型の定義点を分離
    """
    def __init__(self,
                 hub_radius: float,
                 tip_radius: float,
                 num_blades: int,
                 
                 # 形状(ピッチ/弦長)の定義 (例: 20点)
                 r_coords: np.ndarray,
                 pitch_coords_deg: np.ndarray,
                 chord_coords: np.ndarray,
                 
                 # 翼型の定義 (例: 3点)
                 r_coords_airfoil_def: np.ndarray,
                 airfoil_names: list[str],
                 
                 duct_length: float = 0.0,
                 duct_lip_radius: float = 0.0
                 ):
        
        if not (len(r_coords) == len(pitch_coords_deg) == len(chord_coords)):
            raise ValueError("r_coords, pitch_coords_deg, chord_coords の長さが一致しません。")
        if not (len(r_coords_airfoil_def) == len(airfoil_names)):
            raise ValueError("r_coords_airfoil_def と airfoil_names の長さが一致しません。")

        self.hub_radius = hub_radius
        self.tip_radius = tip_radius
        self.num_blades = num_blades
        
        self.diameter = tip_radius * 2.0
        self.duct_length = duct_length
        self.duct_lip_radius = duct_lip_radius

        # 補間用にデータを保持
        # 形状補間用 (線形補間)
        self._r_coords_geom = r_coords
        self._pitch_coords_deg = pitch_coords_deg
        self._chord_coords = chord_coords
        
        # 翼型選択用 (最近傍法)
        self._r_coords_airfoil = r_coords_airfoil_def
        self._airfoil_names = airfoil_names

    def get_pitch_deg(self, r: float) -> float:
        """指定した半径 r でのピッチ角 (度) を線形補間して取得"""
        return float(np.interp(r, self._r_coords_geom, self._pitch_coords_deg))

    def get_chord(self, r: float) -> float:
        """指定した半径 r でのコード長 (m) を線形補間して取得"""
        return float(np.interp(r, self._r_coords_geom, self._chord_coords))
    
    def get_airfoil_name(self, r: float) -> str:
        """
        指定した半径 r で使用する翼型名を取得 (最も近い定義点のものを返す)
        """
        # [修正] 翼型専用の半径座標を参照する
        idx = np.argmin(np.abs(self._r_coords_airfoil - r))
        return self._airfoil_names[idx]
