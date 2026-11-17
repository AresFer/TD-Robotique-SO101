# -*- coding: utf-8 -*-
"""
ik_precise.py — correctif de lerobot_IK (trouvé pendant l'analyse, voir rapport/04).

Constat : lerobot_IK appelle robot.ikine_LM(..., tol=1e-3). Dans le solveur (core/ik.cpp),
l'arrêt se fait quand  E = ½ eᵀ·We·e < tol , e = erreur de pose 6D (m et rad).
Avec tol = 1e-3, une erreur de POSITION seule de |e| < √(2·10⁻³) ≈ 45 mm est déjà jugée
« convergée » : pour un déplacement de 2 à 4 cm, lerobot_IK renvoie… la configuration de
départ, avec success = True. C'est exactement ce que montre pid_ik_sim.py :
« IK success : True — Erreur IK seule [mm] : 28.28 ».

Correctif : même appel, même convention d'angles, mais tolérance 1e-10 (≈ 0.01 mm), plus
d'itérations, et le lissage à 0.1 rad par appel devient optionnel.

Utilisation (remplace l'import) :
    from ik_precise import lerobot_IK_precis as lerobot_IK
"""
import numpy as np
from scipy.spatial.transform import Rotation as R


def lerobot_IK_precis(q_now, target_pose, robot, tol=1e-10, ilimit=60, slimit=5, pas_max=None):
    q_now = np.asarray(q_now, float)
    x, y, z, roll, pitch, yaw = target_pose
    T = np.eye(4)
    T[:3, :3] = R.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    T[:3, 3] = [x, y, z]
    sol = robot.ikine_LM(Tep=T, q0=q_now, ilimit=ilimit, slimit=slimit, tol=tol)
    if not sol.success:
        return -1 * np.ones(len(q_now)), False
    q = np.array(sol.q, float)
    if pas_max is not None:                       # comportement « téléopération » d'origine
        q = q_now + np.clip(q - q_now, -pas_max, pas_max)
    return q, True
