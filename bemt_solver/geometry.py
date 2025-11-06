# bemt_solver/geometry.py
import numpy as np

class Propeller:
    """
    プロペラの幾何学的形状を定義するクラス。
    """
    def __init__(self,
                 hub_radius: float,
                 tip_radius: float,
                 num_blades: int,
                 r_coords: np.ndarray,
                 pitch_coords_deg: np.ndarray,
                 chord_coords: np.ndarray,
                 airfoil_names: list[str], # ◀ [修正] str から list[str] へ
                 duct_length: float = 0.0,
                 duct_lip_radius: float = 0.0
                 ):
        
        # 渡される配列/リストの長さがすべて同じかチェック
        if not (len(r_coords) == len(pitch_coords_deg) == len(chord_coords) == len(airfoil_names)):
            raise ValueError("r_coords, pitch_coords_deg, chord_coords, airfoil_names の長さが一致しません。")

        self.hub_radius = hub_radius
        self.tip_radius = tip_radius
        self.num_blades = num_blades
        
        self.diameter = tip_radius * 2.0
        self.duct_length = duct_length
        self.duct_lip_radius = duct_lip_radius

        # 補間用にデータを保持
        self._r_coords = r_coords
        self._pitch_coords_deg = pitch_coords_deg
        self._chord_coords = chord_coords
        self._airfoil_names = airfoil_names # ◀ [修正]

    def get_pitch_deg(self, r: float) -> float:
        """指定した半径 r でのピッチ角 (度) を補間して取得"""
        return float(np.interp(r, self._r_coords, self._pitch_coords_deg))

    def get_chord(self, r: float) -> float:
        """指定した半径 r でのコード長 (m) を補間して取得"""
        return float(np.interp(r, self._r_coords, self._chord_coords))
    
    def get_airfoil_name(self, r: float) -> str:
        """
        指定した半径 r で使用する翼型名を取得 (最も近い定義点のものを返す)
        """
        # r_coords との差が最小になるインデックスを見つける
        idx = np.argmin(np.abs(self._r_coords - r))
        return self._airfoil_names[idx]