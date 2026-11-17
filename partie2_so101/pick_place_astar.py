# -*- coding: utf-8 -*-
"""
Pick & place A -> B avec PLANIFICATION A* (évitement d'obstacles) + suivi LQR (pyro), bras SO-101.

Principe
--------
1. Planification (astar_planner.py) :
   - A* cherche le chemin le plus court de la pince dans une grille 3D (1 cm) en évitant
     la table et les obstacles (boîtes) ; une case n'est retenue que si le bras peut l'atteindre
     (IK) sans qu'aucune de ses pièces ne touche un obstacle (collisions MuJoCo).
   - Le chemin est lissé (raccourcis en ligne droite) puis converti en angles articulaires.
2. Commande : chaque articulation = double intégrateur q'' = u, gain LQR K calculé par pyro
   (Riccati) à partir des matrices Q et R ; suivi de trajectoire  u = K (x_ref(t) - x) + ddq_ref(t).
3. La position q de l'état LQR est envoyée aux servos (MuJoCo et/ou bras réel).

Séquence : ouvrir pince -> lever -> [A*] au-dessus de A -> descendre -> fermer -> remonter
           -> [A*] au-dessus de B -> descendre -> ouvrir -> remonter -> [A*] retour au départ.

Lancer :  python examples/pick_place_astar.py   (le script se place tout seul à la racine du dépôt)
"""
import json
import os
import sys
import time

import numpy as np
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer

from pyro.dynamic.statespace import StateSpaceSystem
from pyro.analysis.costfunction import QuadraticCostFunction
from pyro.control.lqr import synthesize_lqr_controller

os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # racine du dépôt
sys.path.insert(0, os.path.join(os.getcwd(), "examples"))
from astar_planner import ArmPlanner, JOINT_NAMES                        # noqa: E402

np.set_printoptions(precision=3, suppress=True, linewidth=200)

# =============================================================================
# 1. CONFIGURATION
# =============================================================================
MODE = "real"           # "sim"   : MuJoCo seulement
                       # "real"  : MuJoCo + vrai bras
                       # "teach" : bras réel couple coupé, on enregistre A et B à la main
PORT = "COM12"
CALIB_FILE = "examples/main_follower.json"
POINTS_FILE = "examples/points_AB.json"     # créé par MODE = "teach" (sinon points par défaut)
XML_PATH = "examples/scene_so101.xml"

# Points par défaut (si POINTS_FILE n'existe pas) : position de la pince dans le repère MuJoCo (m)
#   x vers l'avant du robot, y vers sa gauche, z vers le haut, origine sous la base
POINT_A = [0.25, -0.12, 0.02]
POINT_B = [0.22, 0.15, 0.02]

# --- Obstacles (boîtes) dans le repère MuJoCo -------------------------------------------
# OBSTACLE_AUTO = True : une boîte de 4 x 4 x 10 cm posée sur la table au milieu de A et B.
# Sinon, liste de boîtes : centre (x, y, z) et demi-tailles (dx, dy, dz) en mètres.
# Les coordonnées de A et B sont affichées au démarrage pour t'aider à placer les obstacles.
OBSTACLE_AUTO = False
OBSTACLES = [
     {"centre": [0.25, 0.05, 0.05], "demi_taille": [0.02, 0.02, 1]},
]

# --- Planification -------------------------------------------------------------
APPROACH_DZ = 0.06     # hauteur d'approche au-dessus de A et B (m)
GRID_RES = 0.01        # taille des cases de la grille A* (m)
MARGIN = 0.025         # marge de sécurité autour des obstacles pour la pince (m)
CLEARANCE = 0.03       # hauteur minimale de la pince au-dessus de la table pendant les transferts (m)
COMPARE_DIJKSTRA = False   # True : relance chaque planification avec Dijkstra (comparaison, ~20x plus lent)

# --- Pince ---------------------------------------------------------------------
GRIP_OPEN, GRIP_CLOSED = 1.0, 0.0

# --- Réglages LQR --------------------------------------------------------------
# Critère minimisé :  J = ∫ ( xᵀ Q x + uᵀ R u ) dt
#   état    x = [q1..q6 , dq1..dq6]   (12)  -> Q est 12 x 12
#   commande u = [ddq1..ddq6]          (6)  -> R est  6 x  6
# Ordre : 1 base, 2 épaule, 3 coude, 4 flexion poignet, 5 rotation poignet, 6 pince
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

ACC_MAX = 10.0          # accélération max des trajectoires (rad/s²)
VEL_MAX = 10.0         # vitesse max des trajectoires (rad/s)
DT = 0.02              # période de la boucle de commande (s) -> 50 Hz
POS_TOL = 0.02         # tolérance pour considérer une étape terminée (rad)
STEP_TIMEOUT = 8.0     # temps max en plus de la durée prévue d'une étape (s)
TRACK_ERR_MAX = 0.5    # MODE real : arrêt si |q_mesuré - q_consigne| > ce seuil (rad), hors pince
SHOW_PYRO_STEP = False # True : affiche la réponse indicielle LQR (pyro) avant de démarrer

Q_HOME = np.array([-0.98, 1.09, 0.67, -1.63, 1.52, 1.0])  # pose de départ en simulation (bras replié sur le côté)
N_JOINTS = 6


# =============================================================================
# 2. POINTS, OBSTACLES, PLANIFICATION
# =============================================================================
def load_points(planner):
    """Configurations articulaires de A et B (fichier teach, ou points par défaut par IK)."""
    if os.path.exists(POINTS_FILE):
        if os.path.exists(CALIB_FILE) and os.path.getmtime(POINTS_FILE) < os.path.getmtime(CALIB_FILE):
            raise RuntimeError(
                f"{POINTS_FILE} a été enregistré AVANT la calibration actuelle ({CALIB_FILE}) : "
                "ses angles ne correspondent plus au bras.\n"
                "  -> Refais MODE = \"teach\" pour réenregistrer A et B.")
        with open(POINTS_FILE) as f:
            pts = json.load(f)
        qA, qB = np.array(pts["A"], float), np.array(pts["B"], float)
        print(f"Points A/B chargés depuis {POINTS_FILE}")
    else:
        qs = []
        for p in (POINT_A, POINT_B):
            seed = np.array([np.arctan2(-p[1], p[0]), 0.6, 0.9, -1.3, 1.52, GRIP_OPEN])
            planner._context(seed, seed)
            q, ok = planner.ik(np.array(p, float), np.array([0, 0, -1.0]), seed, w_axis=0.0, iters=300)
            if not ok:
                raise ValueError(f"Point {p} hors de portée : change POINT_A / POINT_B.")
            qs.append(q)
        qA, qB = qs
    qA[5] = qB[5] = GRIP_OPEN
    return qA, qB


def make_obstacles(pA, pB, table_z):
    if not OBSTACLE_AUTO:
        return OBSTACLES
    mid = (pA + pB) / 2
    return [{"centre": [float(mid[0]), float(mid[1]), float(table_z + 0.05)], "demi_taille": [0.02, 0.02, 0.05]}]


def above(planner, q, dz):
    """Configuration dz mètres au-dessus de q (même direction de pince)."""
    planner._context(q, q)
    p, a = planner.fk(q)
    q_up, ok = planner.ik_robust(p + np.array([0, 0, dz]), a, q)
    if not ok:
        raise ValueError(f"Impossible de monter de {dz * 100:.0f} cm au-dessus de {np.round(p, 3)}")
    return q_up


def straight(planner, q_from, q_to):
    """Segment en ligne droite (montée / descente / pince seule)."""
    planner._context(q_from, q_to)
    p0, _ = planner.fk(q_from)
    p1, _ = planner.fk(q_to)
    if np.linalg.norm(p1 - p0) < 1e-3:                 # pince seule : interpolation articulaire
        return np.array([q_from, q_to], float)
    try:
        return planner.joint_path([p0, p1], q_from, q_to)
    except RuntimeError as e:                           # ligne droite bloquée : on passe par A*
        print(f"    ligne droite impossible ({e}) -> A*")
        return planner.plan(q_from, q_to)[0]


def joint_move(planner, q_from, q_to, n=60):
    """Interpolation articulaire, refusée si le bras touche un obstacle en chemin."""
    path = np.linspace(np.asarray(q_from, float), np.asarray(q_to, float), n)
    for q in path:
        if planner.collides(q):
            raise RuntimeError("Le trajet articulaire depuis la position de départ touche un obstacle : "
                               "déplace le bras (couple coupé) au-dessus de la zone de travail avant de lancer.")
    return path


def build_sequence(planner, q_start, qA, qB):
    """Retourne la liste (nom, chemin articulaire n x 6, temps de maintien)."""
    def g(q, grip):
        q = np.array(q, float); q[5] = grip; return q

    q_s = g(q_start, GRIP_OPEN)
    try:
        q_s_up = above(planner, q_s, APPROACH_DZ)
        lift_ok = True
    except ValueError:                       # bras replié (pince vers le haut, près de la base…)
        lift_ok = False
    for nom, q in (("de départ", q_s), ("A", qA), ("B", qB)):
        if planner.collides(q):
            raise ValueError(f"La position {nom} touche un obstacle : déplace l'obstacle (OBSTACLES) ou le point.")
    qA_up, qB_up = above(planner, qA, APPROACH_DZ), above(planner, qB, APPROACH_DZ)

    if lift_ok:
        start = [("Ouvrir la pince",      g(q_start, q_start[5]), q_s,    "ligne", 0.3),
                 ("Lever (départ)",       q_s,                    q_s_up, "ligne", 0.1),
                 ("A* -> au-dessus de A", q_s_up,                 qA_up,  "astar", 0.2)]
        end = [("A* -> retour",       qB_up,  q_s_up, "astar", 0.1),
               ("Descente au départ", q_s_up, q_s,    "ligne", 0.5)]
    else:
        print("  Position de départ trop repliée pour monter en ligne droite : "
              "trajet articulaire direct vers le dessus de A (vérifié contre les obstacles)")
        start = [("Ouvrir la pince",            g(q_start, q_start[5]), q_s,   "ligne", 0.3),
                 ("Déplier -> au-dessus de A", q_s,                    qA_up, "joint", 0.2)]
        end = [("Retour au départ (articulaire)", qB_up, q_s, "joint", 0.5)]

    steps = start + [  # (nom, départ, arrivée, type, maintien)
        ("Descente sur A",       qA_up,                  qA,                      "ligne", 0.3),
        ("Fermer la pince (A)",  qA,                     g(qA, GRIP_CLOSED),      "ligne", 0.8),
        ("Remonter de A",        g(qA, GRIP_CLOSED),     g(qA_up, GRIP_CLOSED),   "ligne", 0.2),
        ("A* -> au-dessus de B", g(qA_up, GRIP_CLOSED),  g(qB_up, GRIP_CLOSED),   "astar", 0.2),
        ("Descente sur B",       g(qB_up, GRIP_CLOSED),  g(qB, GRIP_CLOSED),      "ligne", 0.3),
        ("Ouvrir la pince (B)",  g(qB, GRIP_CLOSED),     qB,                      "ligne", 0.8),
        ("Remonter de B",        qB,                     qB_up,                   "ligne", 0.2),
    ] + end
    seq, waypoints = [], []
    for name, q0, q1, kind, hold in steps:
        print(f"  {name}")
        if kind == "astar":
            path, wp = planner.plan(q0, q1, heuristic=True)
            waypoints += wp
            if COMPARE_DIJKSTRA:
                planner.plan(q0, q1, heuristic=False)
        elif kind == "joint":
            path = joint_move(planner, q0, q1)
        else:
            path = straight(planner, q0, q1)
        seq.append((name, path, hold))
    return seq, waypoints


# =============================================================================
# 3. LOI HORAIRE + LQR (pyro)
# =============================================================================
class Reference:
    """Loi horaire le long du chemin articulaire.
    Le chemin est paramétré par sa longueur articulaire cumulée (vitesse régulière même si les
    points sont irrégulièrement espacés) ; profil s = 3t² - 2t³ (départ et arrivée à vitesse nulle).
    Durée choisie pour respecter VEL_MAX et ACC_MAX sur chaque articulation."""

    def __init__(self, path):
        self.path = np.asarray(path, float)
        steps = np.max(np.abs(np.diff(self.path, axis=0)), axis=1) if len(self.path) > 1 else np.zeros(0)
        self.cum = np.concatenate([[0.0], np.cumsum(steps)])
        self.D = self.cum[-1]
        self.T = max(0.3, 1.5 * self.D / VEL_MAX, np.sqrt(6 * self.D / ACC_MAX))

    def q(self, t):
        if self.D < 1e-9:
            return self.path[-1].copy()
        tau = np.clip(t / self.T, 0.0, 1.0)
        d = (3 * tau ** 2 - 2 * tau ** 3) * self.D
        i = min(int(np.searchsorted(self.cum, d, side="right")) - 1, len(self.path) - 2)
        seg = self.cum[i + 1] - self.cum[i]
        f = 0.0 if seg < 1e-12 else (d - self.cum[i]) / seg
        return (1 - f) * self.path[i] + f * self.path[i + 1]

    def __call__(self, t, h=0.01):
        qm, q0, qp = self.q(t - h), self.q(t), self.q(t + h)
        return q0, (qp - qm) / (2 * h), (qp - 2 * q0 + qm) / h ** 2


def build_lqr():
    n = N_JOINTS
    Z, I = np.zeros((n, n)), np.eye(n)
    A = np.block([[Z, I], [Z, Z]])
    B = np.vstack([Z, I])
    ss = StateSpaceSystem(A, B, np.eye(2 * n), np.zeros((2 * n, n)))
    ss.name = "Bras SO-101 (double integrateur)"
    assert Q.shape == (2 * n, 2 * n) and R.shape == (n, n), "dimensions de Q ou R incorrectes"
    assert np.allclose(Q, Q.T) and np.allclose(R, R.T), "Q et R doivent être symétriques"
    assert np.min(np.linalg.eigvalsh(Q)) >= 0, "Q doit être positive"
    assert np.min(np.linalg.eigvalsh(R)) > 0, "R doit être définie positive"
    cf = QuadraticCostFunction(2 * n, n)
    cf.Q, cf.R = Q, R
    ctl = synthesize_lqr_controller(ss, cf)
    return ss, ctl, ctl.K


def pyro_step_response(ss, ctl, q_from, q_to):
    ctl.rbar = np.concatenate([q_to, np.zeros(N_JOINTS)])
    cl = ctl + ss
    cl.x0 = np.concatenate([q_from, np.zeros(N_JOINTS)])
    cl.compute_trajectory(tf=3.0, n=601)
    cl.plot_trajectory("xu")


def lqr_step(x, x_ref, K, a_ref):
    u = np.clip(K @ (x_ref - x) + a_ref, -1.5 * ACC_MAX, 1.5 * ACC_MAX)
    q, dq = x[:N_JOINTS], x[N_JOINTS:]
    dq = np.clip(dq + u * DT, -1.5 * VEL_MAX, 1.5 * VEL_MAX)
    return np.concatenate([q + dq * DT, dq]), u


# =============================================================================
# 4. AFFICHAGE MUJOCO DU CHEMIN
# =============================================================================
def draw_markers(viewer, points, rgba, size=0.006, reset=False):
    scn = viewer.user_scn
    if reset:
        scn.ngeom = 0
    for p in points:
        if scn.ngeom >= scn.maxgeom:
            break
        mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([size, 0, 0]), np.asarray(p, float), np.eye(3).flatten(),
                            np.asarray(rgba, np.float32))
        scn.ngeom += 1


# =============================================================================
# 5. MODE TEACH
# =============================================================================
def teach():
    from lerobot_kinematics import feetech_arm
    arm = feetech_arm(driver_port=PORT, calibration_file=CALIB_FILE)
    arm.arm_hardware.write("Torque_Enable", 0)
    pts = {}
    try:
        for name in ["A", "B"]:
            input(f"\nPlace la pince sur le point {name} (pince ouverte), puis appuie sur Entrée...")
            pts[name] = np.array(arm.feedback(), dtype=float).tolist()
            print(f"{name} = {np.round(pts[name], 3)}")
        with open(POINTS_FILE, "w") as f:
            json.dump(pts, f, indent=2)
        print(f"\nPoints enregistrés dans {POINTS_FILE}")
    finally:
        arm.disconnect()


# =============================================================================
# 6. PROGRAMME PRINCIPAL
# =============================================================================
def main():
    if MODE == "teach":
        teach()
        return

    ss, ctl, K = build_lqr()
    print("Matrice Q :\n", Q, "\nMatrice R :\n", R, "\nGain LQR K = R⁻¹ Bᵀ S :\n", K)

    # --- Bras réel : position de départ mesurée ---
    arm = None
    if MODE == "real":
        from lerobot_kinematics import feetech_arm
        arm = feetech_arm(driver_port=PORT, calibration_file=CALIB_FILE)
        q0 = np.array(arm.feedback(), dtype=float)
        print("Position mesurée au démarrage :", q0)
    else:
        q0 = Q_HOME.copy()

    # --- Planification ---
    probe = ArmPlanner(XML_PATH, [], table_z=0.0)
    qA, qB = load_points(probe)
    pA, _ = probe.fk(qA)
    pB, _ = probe.fk(qB)
    table_z = min(pA[2], pB[2])
    obstacles = make_obstacles(pA, pB, table_z)
    print(f"A = {np.round(pA, 3)}   B = {np.round(pB, 3)}   table à z = {table_z:.3f} m")
    for o in obstacles:
        print(f"Obstacle : centre {np.round(o['centre'], 3)}  demi-tailles {o['demi_taille']}")

    planner = ArmPlanner(XML_PATH, obstacles, table_z=table_z, res=GRID_RES,
                         margin=MARGIN, clearance=CLEARANCE)
    print("Planification...")
    t_plan = time.time()
    seq, waypoints = build_sequence(planner, q0, qA, qB)
    refs = [Reference(path) for _, path, _ in seq]
    print(f"Planification terminée en {time.time() - t_plan:.1f} s ; durée prévue du mouvement "
          f"{sum(r.T for r in refs):.1f} s")

    if SHOW_PYRO_STEP:
        pyro_step_response(ss, ctl, q0, seq[2][1][-1])
        plt.ioff()
        plt.show()

    # --- MuJoCo : scène avec obstacles ---
    mjmodel, mjdata = planner.m, mujoco.MjData(planner.m)
    qpos_idx = planner.qadr
    tips = [planner.fk(q)[0] for _, path, _ in seq for q in path[::4]]

    if arm is not None:
        arm.arm_hardware.write("Goal_Position", np.rad2deg(q0) * np.array([-1, -1, 1, 1, -1, 1]))
        arm.arm_hardware.write("Torque_Enable", 1)

    x = np.concatenate([q0, np.zeros(N_JOINTS)])
    log_t, log_q, log_ref, log_u, log_meas = [], [], [], [], []
    t0, err_since = time.time(), None
    try:
        with mujoco.viewer.launch_passive(mjmodel, mjdata) as viewer:
            draw_markers(viewer, tips, [0.1, 0.8, 0.2, 0.8], 0.004, reset=True)       # chemin (vert)
            draw_markers(viewer, waypoints, [0.1, 0.3, 1.0, 1.0], 0.008)             # points A* lissés (bleu)
            draw_markers(viewer, [pA, pB], [1.0, 0.8, 0.0, 1.0], 0.012)              # A et B (jaune)
            for (name, path, hold), ref in zip(seq, refs):
                print(f"--> {name}  ({ref.T:.1f} s)")
                q_end = path[-1]
                t_step, t_reached = time.time(), None
                while viewer.is_running():
                    loop_start = time.time()
                    t_rel = loop_start - t_step
                    q_r, dq_r, a_r = ref(t_rel)
                    x, u = lqr_step(x, np.concatenate([q_r, dq_r]), K, a_r)

                    mjdata.qpos[qpos_idx] = x[:N_JOINTS]
                    mujoco.mj_forward(mjmodel, mjdata)
                    viewer.sync()

                    q_meas = None
                    if arm is not None:
                        arm.action(x[:N_JOINTS].copy())
                        q_meas = np.array(arm.feedback(), dtype=float)
                        e = np.abs(q_meas[:5] - x[:5])
                        if e.max() > TRACK_ERR_MAX:
                            err_since = err_since or time.time()
                            if time.time() - err_since > 0.5:
                                raise RuntimeError(
                                    f"Écart de suivi {e.max():.2f} rad sur {JOINT_NAMES[int(np.argmax(e))]} : "
                                    f"arrêt de sécurité.\n  consigne = {np.round(x[:6], 2)}\n"
                                    f"  mesure   = {np.round(q_meas, 2)}")
                        else:
                            err_since = None

                    log_t.append(time.time() - t0); log_q.append(x[:N_JOINTS].copy())
                    log_ref.append(q_r.copy()); log_u.append(u.copy())
                    log_meas.append(q_meas if q_meas is not None else np.full(N_JOINTS, np.nan))

                    reached = (t_rel >= ref.T and np.max(np.abs(x[:N_JOINTS] - q_end)) < POS_TOL
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
    fig.suptitle("Pick & place A* + LQR")
    plt.tight_layout()
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
