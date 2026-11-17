# -*- coding: utf-8 -*-
"""
Pick & place A -> B avec un contrôleur LQR (bibliothèque pyro) sur bras SO-101 / SO-100.

Principe
--------
- La cinématique (FK / IK) vient de lerobot_kinematics (code du dépôt).
- La visualisation est la scène MuJoCo du dépôt.
- La trajectoire est calculée dans l'espace CARTÉSIEN (IK tous les 5 mm) : la pince
  monte et descend à la verticale et se déplace à hauteur constante -> elle ne
  traverse jamais la table, quelles que soient les vitesses des articulations.
- Chaque articulation est modélisée par un double intégrateur  q'' = u
  (état x = [q ; dq], commande u = accélération articulaire).
- pyro calcule le gain LQR K (équation de Riccati). Suivi de trajectoire :
      u = K (x_ref(t) - x) + ddq_ref(t)      (LQR + anticipation)
- Un séquenceur enchaîne les consignes depuis la position de départ du bras :
  au-dessus de A -> A -> fermer pince -> remonter -> au-dessus de B -> B ->
  ouvrir pince -> remonter -> retour à la position de départ.
- La position q de l'état LQR est envoyée en consigne aux servos (MuJoCo et/ou bras réel).

Lancer depuis la RACINE du dépôt (Working directory = racine dans PyCharm) :
    python examples/pick_place_trajectoire.py
"""

import contextlib
import io
import json
import os
import time

import numpy as np
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer

from lerobot_kinematics import lerobot_IK, lerobot_FK, get_robot

from pyro.dynamic.statespace import StateSpaceSystem
from pyro.analysis.costfunction import QuadraticCostFunction
from pyro.control.lqr import synthesize_lqr_controller

np.set_printoptions(precision=3, suppress=True, linewidth=200)
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # racine du dépôt

# =============================================================================
# 1. CONFIGURATION  (à adapter)
# =============================================================================
ROBOT = "so101"        # "so101" ou "so100"
MODE = "real"           # "sim"   : MuJoCo seulement
                       # "real"  : MuJoCo + vrai bras (port + calibration requis)
                       # "teach" : bras réel couple coupé, on enregistre A et B à la main
PORT = "COM12"                             # port de la carte Seeed (Gestionnaire de périphériques)
CALIB_FILE = "examples/main_follower.json"  # calibration du bras (format lerobot-joycon)
POINTS_FILE = "examples/points_AB.json"     # points A/B enregistrés par MODE = "teach"

# --- Points A et B ---------------------------------------------------------
# Option 1 (par défaut) : coordonnées cartésiennes dans le repère de l'IK du dépôt
#   x = distance horizontale depuis l'axe de rotation de la base (m)
#   z = hauteur par rapport à l'articulation de l'épaule (m)
#   pan = rotation de la base (rad) pour aller à gauche / à droite
# Option 2 : si POINTS_FILE existe (créé par MODE = "teach"), il est utilisé à la place.
POINT_A = {"pan": +0.6, "x": 0.22, "z": 0.06}
POINT_B = {"pan": -0.6, "x": 0.22, "z": 0.06}
APPROACH_DZ = 0.06     # hauteur d'approche au-dessus de A et B (m)

# --- Réglages LQR -------------------------------------------------------------
# Critère minimisé :  J = ∫ ( xᵀ Q x + uᵀ R u ) dt
#   état    x = [q1..q6 , dq1..dq6]   (12)  -> Q est 12 x 12
#   commande u = [ddq1..ddq6]          (6)  -> R est  6 x  6
# Ordre des articulations : 1 base, 2 épaule, 3 coude, 4 flexion poignet, 5 rotation poignet, 6 pince
#   diagonale de Q, colonnes 1-6  : poids de l'erreur de POSITION (plus grand = correction plus rapide)
#   diagonale de Q, colonnes 7-12 : poids de la VITESSE            (plus grand = plus amorti)
#   diagonale de R               : poids de la COMMANDE           (plus grand = plus doux)
# Q doit être symétrique positive, R symétrique définie positive (diagonale > 0).
#                q1   q2   q3   q4   q5   q6  dq1  dq2  dq3  dq4  dq5  dq6
Q = np.array([[  81,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],   # q1
              [   0,  81,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],   # q2
              [   0,   0,  81,   0,   0,   0,   0,   0,   0,   0,   0,   0],   # q3
              [   0,   0,   0,  81,   0,   0,   0,   0,   0,   0,   0,   0],   # q4
              [   0,   0,   0,   0,  81,   0,   0,   0,   0,   0,   0,   0],   # q5
              [   0,   0,   0,   0,   0,  81,   0,   0,   0,   0,   0,   0],   # q6
              [   0,   0,   0,   0,   0,   0,   1,   0,   0,   0,   0,   0],   # dq1
              [   0,   0,   0,   0,   0,   0,   0,   1,   0,   0,   0,   0],   # dq2
              [   0,   0,   0,   0,   0,   0,   0,   0,   1,   0,   0,   0],   # dq3
              [   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   0,   0],   # dq4
              [   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   0],   # dq5
              [   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1]],  # dq6
             dtype=float)

#              u1  u2  u3  u4  u5  u6
R = np.array([[ 1,  0,  0,  0,  0,  0],   # u1 (ddq base)
              [ 0,  1,  0,  0,  0,  0],   # u2 (ddq épaule)
              [ 0,  0,  1,  0,  0,  0],   # u3 (ddq coude)
              [ 0,  0,  0,  1,  0,  0],   # u4 (ddq flexion poignet)
              [ 0,  0,  0,  0,  1,  0],   # u5 (ddq rotation poignet)
              [ 0,  0,  0,  0,  0,  1]],  # u6 (ddq pince)
             dtype=float)

#ACC_MAX = 2.0
ACC_MAX = 25.0         # saturation accélération (rad/s²)
#VEL_MAX = 0.6
VEL_MAX = 20.0          # saturation vitesse (rad/s)
DT = 0.02              # période de la boucle de commande (s) -> 50 Hz

PATH_STEP = 0.005      # pas de la trajectoire cartésienne (m)

POS_TOL = 0.02         # tolérance pour considérer une consigne atteinte (rad)
STEP_TIMEOUT = 8.0     # temps max par étape (s)
TRACK_ERR_MAX = 0.5    # MODE real : arrêt si |q_mesuré - q_consigne| > ce seuil (rad), hors pince

SHOW_PYRO_STEP = True  # affiche la réponse indicielle LQR calculée par pyro avant de démarrer

# --- Paramètres spécifiques au robot -------------------------------------------
if ROBOT == "so101":
    XML_PATH = "./examples/scene_so101.xml"
    JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
                   "wrist_flex", "wrist_roll", "gripper"]
    Q_HOME = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    GRIP_OPEN, GRIP_CLOSED = 1.0, 0.0
else:
    XML_PATH = "./examples/scene.xml"
    JOINT_NAMES = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]
    Q_HOME = np.array([0.0, -3.14, 3.14, 0.0, -1.57, -0.157])
    GRIP_OPEN, GRIP_CLOSED = 1.0, -0.15

N_JOINTS = 6
robot = get_robot(ROBOT)


# =============================================================================
# 2. CINÉMATIQUE (lerobot_kinematics)
# =============================================================================
def solve_ik(gpos_target, q4_seed, n_interp=40, n_iter=15):
    """IK robuste : on rapproche la cible par petits pas depuis la pose de départ
    (lerobot_IK limite chaque appel à 0.1 rad par articulation).
    gpos = [x, y, z, roll, pitch, yaw] ; q4 = [shoulder_lift, elbow, wrist_flex, wrist_roll]."""
    q4 = np.array(q4_seed, dtype=float)
    g_start = lerobot_FK(q4, robot=robot)
    gpos_target = np.asarray(gpos_target, dtype=float)
    for k in range(1, n_interp + 1):
        g_k = g_start + (gpos_target - g_start) * k / n_interp
        for _ in range(n_iter):
            with contextlib.redirect_stdout(io.StringIO()):   # masque les « IK fails » intermédiaires
                q_new, ok = lerobot_IK(q4, g_k, robot=robot)
            if not ok:
                break
            done = np.max(np.abs(q_new - q4)) < 1e-4
            q4 = np.array(q_new, dtype=float)
            if done:
                break
    err = np.linalg.norm(lerobot_FK(q4, robot=robot)[:3] - gpos_target[:3])
    if err > 5e-3:
        raise ValueError(f"IK : cible {gpos_target[:3]} non atteinte (erreur {err*1000:.1f} mm). "
                         "Point hors de portée ? Change x/z.")
    return q4


def point_to_q(point, grip, q4_seed, dz=0.0):
    """Point {pan, x, z} -> configuration articulaire 6D, orientation de la pince = celle du repos."""
    g_home = lerobot_FK(Q_HOME[1:5], robot=robot)
    gpos = np.array([point["x"], 0.0, point["z"] + dz, *g_home[3:]])
    q4 = solve_ik(gpos, q4_seed)
    return np.array([point["pan"], *q4, grip])


def q_above(q6, dz, grip):
    """Configuration située dz mètres au-dessus de q6 (même orientation de pince)."""
    gpos = lerobot_FK(q6[1:5], robot=robot).copy()
    gpos[2] += dz
    q4 = solve_ik(gpos, q6[1:5])
    return np.array([q6[0], *q4, grip])


def build_waypoints(q_start):
    """Retourne la liste des étapes (nom, q_ref 6D, temps de maintien).
    Le bras part de q_start et y revient : pas de grand mouvement vers une pose de repos."""
    if os.path.exists(POINTS_FILE):
        with open(POINTS_FILE) as f:
            pts = json.load(f)
        qA = np.array(pts["A"], dtype=float)
        qB = np.array(pts["B"], dtype=float)
        qA[5], qB[5] = GRIP_OPEN, GRIP_OPEN
        print(f"Points A/B chargés depuis {POINTS_FILE}")
    else:
        qA = point_to_q(POINT_A, GRIP_OPEN, Q_HOME[1:5])
        qB = point_to_q(POINT_B, GRIP_OPEN, Q_HOME[1:5])

    qA_up = q_above(qA, APPROACH_DZ, GRIP_OPEN)
    qB_up_closed = q_above(qB, APPROACH_DZ, GRIP_CLOSED)
    qA_closed = qA.copy(); qA_closed[5] = GRIP_CLOSED
    qA_up_closed = qA_up.copy(); qA_up_closed[5] = GRIP_CLOSED
    qB_closed = qB.copy(); qB_closed[5] = GRIP_CLOSED
    qB_up = qB_up_closed.copy(); qB_up[5] = GRIP_OPEN
    q_start_open = np.array(q_start, dtype=float); q_start_open[5] = GRIP_OPEN
    q_start_up = q_above(q_start_open, APPROACH_DZ, GRIP_OPEN)

    return [
        ("Ouvrir la pince",        q_start_open, 0.3),
        ("Lever (départ)",         q_start_up,   0.1),
        ("Au-dessus de A",         qA_up,        0.2),
        ("Descente sur A",         qA,           0.3),
        ("Fermer la pince (A)",    qA_closed,    0.8),
        ("Remonter de A",          qA_up_closed, 0.2),
        ("Au-dessus de B",         qB_up_closed, 0.2),
        ("Descente sur B",         qB_closed,    0.3),
        ("Ouvrir la pince (B)",    qB,           0.8),
        ("Remonter de B",          qB_up,        0.2),
        ("Au-dessus du départ",    q_start_up,   0.1),
        ("Retour au départ",       q_start_open, 0.5),
    ]


# =============================================================================
# 2b. TRAJECTOIRE CARTÉSIENNE + LOI HORAIRE
# =============================================================================
def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def cart_path(q_from, q_to):
    """Configurations successives (n x 6) pour que la pince aille en LIGNE DROITE
    de q_from à q_to (position + orientation interpolées, IK à chaque point)."""
    g0 = lerobot_FK(q_from[1:5], robot=robot)
    g1 = lerobot_FK(q_to[1:5], robot=robot)
    dg = g1 - g0
    dg[3:] = wrap(dg[3:])
    r = max(abs(g0[0]), abs(g1[0]))
    dist = np.sqrt(dg[0] ** 2 + dg[2] ** 2 + (r * (q_to[0] - q_from[0])) ** 2)
    n = max(1, int(np.ceil(dist / PATH_STEP)))

    path = [np.array(q_from, dtype=float)]
    q4 = np.array(q_from[1:5], dtype=float)
    for k in range(1, n):
        a = k / n
        try:
            q4 = solve_ik(g0 + a * dg, q4, n_interp=2, n_iter=15)
        except ValueError:                       # point délicat : interpolation articulaire
            q4 = (1 - a) * np.asarray(q_from[1:5]) + a * np.asarray(q_to[1:5])
        pan = (1 - a) * q_from[0] + a * q_to[0]
        grip = (1 - a) * q_from[5] + a * q_to[5]
        path.append(np.array([pan, *q4, grip]))
    path.append(np.array(q_to, dtype=float))
    return np.array(path)


class Reference:
    """Loi horaire polynomiale s = 3t² - 2t³ le long du chemin : départ et arrivée à vitesse nulle.
    Durée choisie pour respecter VEL_MAX et ACC_MAX sur chaque articulation."""

    def __init__(self, path):
        self.path = path
        self.m = len(path) - 1
        dq = np.max(np.abs(np.diff(path, axis=0))) if self.m > 0 else 0.0
        self.T = max(0.3,
                     1.5 * self.m * dq / VEL_MAX,
                     np.sqrt(6 * self.m * dq / ACC_MAX))

    def q(self, t):
        tau = np.clip(t / self.T, 0.0, 1.0)
        sigma = (3 * tau ** 2 - 2 * tau ** 3) * self.m
        i = min(int(sigma), self.m - 1) if self.m > 0 else 0
        if self.m == 0:
            return self.path[0].copy()
        f = sigma - i
        return (1 - f) * self.path[i] + f * self.path[i + 1]

    def __call__(self, t, h=0.01):
        """Retourne (q_ref, dq_ref, ddq_ref) à l'instant t."""
        qm, q0, qp = self.q(t - h), self.q(t), self.q(t + h)
        return q0, (qp - qm) / (2 * h), (qp - 2 * q0 + qm) / h ** 2


# =============================================================================
# 3. LQR (pyro)
# =============================================================================
def build_lqr():
    """Double intégrateur par articulation : x = [q ; dq], u = ddq.
    Retourne (système pyro, contrôleur pyro, gain K)."""
    n = N_JOINTS
    Z, I = np.zeros((n, n)), np.eye(n)
    A = np.block([[Z, I], [Z, Z]])
    B = np.vstack([Z, I])
    C = np.eye(2 * n)
    D = np.zeros((2 * n, n))
    ss = StateSpaceSystem(A, B, C, D)
    ss.name = "Bras SO (double integrateur)"
    ss.state_label = [f"q{i}" for i in range(n)] + [f"dq{i}" for i in range(n)]
    ss.input_label = [f"ddq{i}" for i in range(n)]

    # Vérification des matrices de pondération
    assert Q.shape == (2 * n, 2 * n), f"Q doit être {2 * n} x {2 * n}"
    assert R.shape == (n, n), f"R doit être {n} x {n}"
    assert np.allclose(Q, Q.T) and np.allclose(R, R.T), "Q et R doivent être symétriques"
    assert np.min(np.linalg.eigvalsh(Q)) >= 0, "Q doit être positive (valeurs propres >= 0)"
    assert np.min(np.linalg.eigvalsh(R)) > 0, "R doit être définie positive (valeurs propres > 0)"

    cf = QuadraticCostFunction(2 * n, n)
    cf.Q = Q
    cf.R = R

    ctl = synthesize_lqr_controller(ss, cf)   # résout Riccati, u = K (r - x)
    return ss, ctl, ctl.K


def pyro_step_response(ss, ctl, q_from, q_to):
    """Réponse indicielle LQR non saturée simulée par pyro (pour le compte rendu)."""
    ctl.rbar = np.concatenate([q_to, np.zeros(N_JOINTS)])   # consigne
    cl = ctl + ss                                            # boucle fermée pyro
    cl.x0 = np.concatenate([q_from, np.zeros(N_JOINTS)])
    cl.compute_trajectory(tf=3.0, n=601)
    cl.plot_trajectory("xu")


def lqr_step(x, x_ref, K, a_ref=0.0):
    """Une période de commande : u = K (x_ref - x) + a_ref, saturations, intégration d'Euler."""
    u = K @ (x_ref - x) + a_ref
    u = np.clip(u, -1.5 * ACC_MAX, 1.5 * ACC_MAX)
    q, dq = x[:N_JOINTS], x[N_JOINTS:]
    dq = np.clip(dq + u * DT, -1.5 * VEL_MAX, 1.5 * VEL_MAX)
    q = q + dq * DT
    return np.concatenate([q, dq]), u


# =============================================================================
# 4. MODE TEACH : enregistrer A et B à la main sur le vrai bras
# =============================================================================
def teach():
    from lerobot_kinematics import feetech_arm
    arm = feetech_arm(driver_port=PORT, calibration_file=CALIB_FILE)
    arm.arm_hardware.write("Torque_Enable", 0)       # bras libre
    pts = {}
    try:
        for name in ["A", "B"]:
            input(f"\nPlace la pince sur le point {name} (pince ouverte), puis appuie sur Entrée...")
            q = np.array(arm.feedback(), dtype=float)
            pts[name] = q.tolist()
            print(f"{name} = {q}")
        with open(POINTS_FILE, "w") as f:
            json.dump(pts, f, indent=2)
        print(f"\nPoints enregistrés dans {POINTS_FILE}")
    finally:
        arm.disconnect()


# =============================================================================
# 5. BOUCLE PRINCIPALE
# =============================================================================
def main():
    if MODE == "teach":
        teach()
        return

    ss, ctl, K = build_lqr()
    print("Matrice Q (12 x 12) :\n", Q)
    print("Matrice R (6 x 6) :\n", R)
    print("Gain LQR K = R⁻¹ Bᵀ S (6 x 12) :\n", K)

    # --- MuJoCo (scène du dépôt) ---
    mjmodel = mujoco.MjModel.from_xml_path(XML_PATH)
    mjdata = mujoco.MjData(mjmodel)
    qpos_idx = np.array([mjmodel.jnt_qposadr[mjmodel.joint(n).id] for n in JOINT_NAMES])

    # --- Bras réel ---
    arm = None
    if MODE == "real":
        from lerobot_kinematics import feetech_arm
        arm = feetech_arm(driver_port=PORT, calibration_file=CALIB_FILE)
        q0 = np.array(arm.feedback(), dtype=float)   # on part de la position réelle : pas de saut
        arm.arm_hardware.write("Goal_Position", np.rad2deg(q0) * np.array([-1, -1, 1, 1, -1, 1]))
        arm.arm_hardware.write("Torque_Enable", 1)   # couple activé sur la position actuelle
        print("Position mesurée au démarrage :", q0)
    else:
        q0 = Q_HOME.copy()

    waypoints = build_waypoints(q0)
    print("Calcul des trajectoires cartésiennes...")
    refs, q_prev = [], q0.copy()
    for name, q_ref, hold in waypoints:
        ref = Reference(cart_path(q_prev, q_ref))
        refs.append(ref)
        print(f"  {name:22s} {len(ref.path):3d} pts  {ref.T:4.1f} s  q_fin = {q_ref}")
        q_prev = q_ref

    if SHOW_PYRO_STEP:
        pyro_step_response(ss, ctl, q0, waypoints[1][1])
        plt.ioff()
        plt.show()   # ferme la fenêtre pour lancer la séquence

    x = np.concatenate([q0, np.zeros(N_JOINTS)])
    log_t, log_q, log_ref, log_u, log_meas = [], [], [], [], []
    t0 = time.time()
    err_since = None

    try:
        with mujoco.viewer.launch_passive(mjmodel, mjdata) as viewer:
            for (name, q_ref, hold), ref in zip(waypoints, refs):
                print(f"--> {name}  ({ref.T:.1f} s)")
                t_step, t_reached = time.time(), None

                while viewer.is_running():
                    loop_start = time.time()
                    t_rel = loop_start - t_step
                    q_r, dq_r, a_r = ref(t_rel)
                    x_ref = np.concatenate([q_r, dq_r])
                    x, u = lqr_step(x, x_ref, K, a_r)

                    # Consigne vers la simulation
                    mjdata.qpos[qpos_idx] = x[:N_JOINTS]
                    mujoco.mj_forward(mjmodel, mjdata)
                    viewer.sync()

                    # Consigne vers le vrai bras + surveillance
                    q_meas = None
                    if arm is not None:
                        arm.action(x[:N_JOINTS].copy())
                        q_meas = np.array(arm.feedback(), dtype=float)
                        err = np.max(np.abs(q_meas[:5] - x[:5]))   # pince exclue
                        if err > TRACK_ERR_MAX:
                            err_since = err_since or time.time()
                            if time.time() - err_since > 0.5:
                                raise RuntimeError(
                                    f"Écart de suivi {err:.2f} rad : arrêt de sécurité.\n"
                                    f"  articulation : {JOINT_NAMES[int(np.argmax(np.abs(q_meas[:5] - x[:5])))]}\n"
                                    f"  consigne = {np.round(x[:6], 2)}\n  mesure   = {np.round(q_meas, 2)}")
                        else:
                            err_since = None

                    log_t.append(time.time() - t0); log_q.append(x[:N_JOINTS].copy())
                    log_ref.append(q_r.copy()); log_u.append(u.copy())
                    log_meas.append(q_meas if q_meas is not None else np.full(N_JOINTS, np.nan))

                    # Consigne atteinte ?
                    reached = (t_rel >= ref.T
                               and np.max(np.abs(x[:N_JOINTS] - q_ref)) < POS_TOL
                               and np.max(np.abs(x[N_JOINTS:])) < 0.05)
                    if reached and t_reached is None:
                        t_reached = time.time()
                    if t_reached is not None and time.time() - t_reached >= hold:
                        break
                    if t_rel > ref.T + STEP_TIMEOUT:
                        print(f"   ! délai dépassé pour « {name} », étape suivante")
                        break

                    dt_left = DT - (time.time() - loop_start)
                    if dt_left > 0:
                        time.sleep(dt_left)

                if not viewer.is_running():
                    break
            print("Séquence terminée.")
    except KeyboardInterrupt:
        print("Interrompu par l'utilisateur.")
    finally:
        if arm is not None:
            arm.disconnect()

    plot_logs(log_t, log_q, log_ref, log_u, log_meas)


def plot_logs(t, q, ref, u, meas):
    if not t:
        return
    t, q, ref, u, meas = map(np.array, (t, q, ref, u, meas))
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    for j in range(N_JOINTS):
        axes[0].plot(t, q[:, j], label=JOINT_NAMES[j])
        axes[0].plot(t, ref[:, j], "--", color=axes[0].lines[-1].get_color(), alpha=0.5)
        if not np.all(np.isnan(meas[:, j])):
            axes[1].plot(t, meas[:, j] - q[:, j], label=JOINT_NAMES[j])
        axes[2].plot(t, u[:, j], label=JOINT_NAMES[j])
    axes[0].set_ylabel("q (rad)\n— état, -- consigne")
    axes[1].set_ylabel("erreur mesurée (rad)")
    axes[2].set_ylabel("u = ddq (rad/s²)")
    axes[2].set_xlabel("temps (s)")
    axes[0].legend(loc="upper right", fontsize=8)
    for ax in axes:
        ax.grid(True)
    fig.suptitle("Pick & place LQR")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
