# -*- coding: utf-8 -*-
"""
Remplaçant « simulation seule » de la bibliothèque lerobot-kinematics (box2ai-robotics).

Pourquoi ?  lerobot-kinematics embarque une extension C (fknm) qu'il faut compiler. Pour
régénérer les figures sur n'importe quelle machine (CI, PC sans compilateur), ce module
reproduit À L'IDENTIQUE les fonctions utilisées par nos scripts :

    get_robot("so100" | "so101"), lerobot_FK(q4, robot), lerobot_IK(q4, gpos, robot)

- même chaîne ETS (mêmes translations tx/tz, mêmes axes Ry/Rx, mêmes butées qlim),
- même appel ikine_LM(ilimit=10, slimit=2, tol=1e-3),
- même lissage smooth_joint_motion (0.1 rad max par appel),
- mêmes arrondis à 3 décimales dans le calcul des angles d'Euler.

La seule différence : la classe ET vient de Robotics Toolbox for Python (P. Corke), dont
lerobot-kinematics est un fork. Le pilote matériel feetech_arm n'est PAS fourni ici : sur le
vrai bras, installez la vraie bibliothèque (voir rapport/03_installation_calibration.md).
"""
import math
from math import sqrt

import numpy as np
from roboticstoolbox import ET
from scipy.spatial.transform import Rotation as R

__all__ = ["get_robot", "lerobot_FK", "lerobot_IK", "feetech_arm"]


def atan2(a, b):
    return round(math.atan2(a, b), 3)


def cos(x):
    return round(math.cos(x), 3)


def create_so100():
    so100 = (ET.tx(0.02943) * ET.tz(0.05504) * ET.Ry() *
             ET.tx(0.1127) * ET.tz(-0.02798) * ET.Ry() *
             ET.tx(0.13504) * ET.tz(0.00519) * ET.Ry() *
             ET.tx(0.0593) * ET.tz(0.00996) * ET.Rx())
    so100.qlim = np.array([[-3.14158, -0.2, -1.5, -3.14158],
                           [0.2, 3.14158, 1.5, 3.14158]])
    return so100


def create_so101():
    so101 = (ET.tx(0.02943) * ET.tz(0.05504) * ET.Ry() *
             ET.tz(0.1127) * ET.tx(0.02798) * ET.Ry() *
             ET.tx(0.13504) * ET.tz(0.00519) * ET.Ry() *
             ET.tx(0.0593) * ET.tz(0.00996) * ET.Rx())
    so101.qlim = np.array([[-1.57, -1.57, -1.5, -3.14158],
                           [1.57, 1.57, 1.5, 3.14158]])
    return so101


def get_robot(robot="so100"):
    if robot == "so100":
        return create_so100()
    if robot == "so101":
        return create_so101()
    print(f"Sorry, we don't support {robot} robot now")
    return None


def lerobot_FK(qpos_data, robot):
    if len(qpos_data) != robot.n:
        raise Exception("The dimensions of qpose_data are not the same as the robot joint dimensions")
    T = robot.fkine(np.asarray(qpos_data, float))
    X, Y, Z = T.t
    Rm = T.R
    beta = atan2(-Rm[2, 0], sqrt(Rm[0, 0] ** 2 + Rm[1, 0] ** 2))
    if cos(beta) != 0:
        alpha = atan2(Rm[1, 0] / cos(beta), Rm[0, 0] / cos(beta))
        gamma = atan2(Rm[2, 1] / cos(beta), Rm[2, 2] / cos(beta))
    else:
        alpha = 0
        gamma = atan2(Rm[0, 1], Rm[1, 1])
    return np.array([X, Y, Z, gamma, beta, alpha])


def lerobot_IK(q_now, target_pose, robot):
    if len(q_now) != robot.n:
        raise Exception("The dimensions of qpose_data are not the same as the robot joint dimensions")
    x, y, z, roll, pitch, yaw = target_pose
    T = np.eye(4)
    T[:3, :3] = R.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_matrix()
    T[:3, 3] = [x, y, z]
    sol = robot.ikine_LM(Tep=T, q0=np.asarray(q_now, float), ilimit=10, slimit=2, tol=1e-3)
    if sol.success:
        return smooth_joint_motion(q_now, np.array(sol.q, float)), True
    print("IK fails")
    return -1 * np.ones(len(q_now)), False


def smooth_joint_motion(q_now, q_new, max_joint_change=0.1):
    for i in range(len(q_new)):
        delta = q_new[i] - q_now[i]
        if abs(delta) > max_joint_change:
            delta = np.sign(delta) * max_joint_change
        q_new[i] = q_now[i] + delta
    return q_new


class feetech_arm:  # pragma: no cover - placeholder, remplacé par le jumeau numérique
    def __init__(self, *a, **k):
        raise RuntimeError("Pilote Feetech indisponible en simulation : utiliser outils.jumeau.FakeFeetechArm")
