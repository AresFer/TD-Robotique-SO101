# -*- coding: utf-8 -*-
"""
Q6 — LQR : aller d'un point A à un point B, puis « parcours en étoile », bras SO-101.

Deux lectures possibles de « parcours en étoile » sont implémentées :
  PATTERN = "pentagramme" : la pince DESSINE une étoile à 5 branches {5/2} dans un plan
                            horizontal (sommets 0 -> 2 -> 4 -> 1 -> 3 -> 0) ;
  PATTERN = "rayons"      : parcours en étoile au sens « hub » : depuis un centre C, aller-retour
                            vers N points répartis sur un cercle (C -> P1 -> C -> P2 -> C ...).

Chaîne de commande (même architecture que pick_place_astar.py) :
  segments en LIGNE DROITE dans l'espace cartésien  -> IK tous les 5 mm (ArmPlanner.joint_path,
  pince verticale)  -> loi horaire s = 3t² - 2t³ par segment (arrêt à chaque sommet, car les
  angles de l'étoile sont vifs)  -> suivi LQR pyro u = K (x_ref - x) + ddq_ref  -> servos.

Lancer depuis la racine de lerobot-kinematics (script copié dans examples/) :
    python examples/lqr_etoile.py
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

# =============================================================================
# 1. CONFIGURATION
# =============================================================================
MODE = "sim"            # "sim" : MuJoCo seul ; "real" : MuJoCo + vrai bras
PORT = "COM12"
CALIB_FILE = "examples/main_follower.json"
XML_PATH = "examples/scene_so101.xml"

PATTERN = "pentagramme"   # "pentagramme" ou "rayons"
CENTRE = [0.24, 0.0]      # centre de l'étoile dans le plan (x, y) du repère MuJoCo (m)
RAYON = 0.06              # rayon des sommets (m)
Z_DESSIN = 0.04           # hauteur de la pince pendant le dessin (m) — pince verticale atteignable jusqu'à ~6 cm
N_RAYONS = 6              # nombre de branches pour PATTERN = "rayons"
A_DEPART = [0.20, -0.15, 0.05]   # point A (départ du parcours A -> B -> étoile)

# LQR : mêmes conventions que pick_place_astar.py (double intégrateur par articulation)
Q_POS, Q_VIT, R_U = 81.0, 1.0, 1.0
ACC_MAX = 4.0          # accélération max des trajectoires (rad/s²)
VEL_MAX = 1.5          # vitesse max des trajectoires (rad/s)
DT = 0.02              # 50 Hz
POS_TOL = 0.01
HOLD = 0.15            # pause à chaque sommet (s)
STEP_TIMEOUT = 5.0
TRACK_ERR_MAX = 0.5
N_JOINTS = 6
Q_HOME = np.array([0.0, -1.0, 1.0, 1.2, 1.52, 0.0])   # pose de départ en simulation (pince à 6 cm)


# =============================================================================
# 2. GÉOMÉTRIE DU PARCOURS
# =============================================================================
def sommets_etoile():
    """Liste des points (x, y, z) à relier en ligne droite."""
    cx, cy = CENTRE
    if PATTERN == "pentagramme":
        ang = np.pi / 2 + 2 * np.pi * np.arange(5) / 5
        pts = [np.array([cx + RAYON * np.cos(a), cy + RAYON * np.sin(a), Z_DESSIN]) for a in ang]
        ordre = [0, 2, 4, 1, 3, 0]
        return [pts[i] for i in ordre]
    c = np.array([cx, cy, Z_DESSIN])
    out = [c]
    for k in range(N_RAYONS):
        a = 2 * np.pi * k / N_RAYONS
        out += [c + RAYON * np.array([np.cos(a), np.sin(a), 0]), c]
    return out


def construire_parcours(planner, q0):
    """Segments (nom, chemin articulaire) : départ -> A -> B (= 1er sommet) -> étoile."""
    bas = np.array([0, 0, -1.0])
    planner._context(q0, q0)
    qA, okA = planner.ik_robust(np.array(A_DEPART), bas, q0)
    if not okA:
        raise ValueError(f"A = {A_DEPART} inatteignable avec la pince verticale")
    sommets = sommets_etoile()
    segments = [("Départ -> A (articulaire)", np.linspace(q0, qA, 40))]
    q_prev, p_prev = qA, np.array(A_DEPART)
    for k, p in enumerate(sommets):
        planner._context(q_prev, q_prev)
        q_cible, ok = planner.ik_robust(p, bas, q_prev)
        if not ok:
            raise ValueError(f"sommet {np.round(p, 3)} inatteignable")
        chemin = planner.joint_path([p_prev, p], q_prev, q_cible)
        nom = "A -> B (ligne droite)" if k == 0 else f"branche {k}"
        segments.append((nom, chemin))
        q_prev, p_prev = chemin[-1], p
    return segments, sommets


# =============================================================================
# 3. LOI HORAIRE + LQR  (repris de pick_place_astar.py)
# =============================================================================
class Reference:
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
    ss = StateSpaceSystem(np.block([[Z, I], [Z, Z]]), np.vstack([Z, I]), np.eye(2 * n), np.zeros((2 * n, n)))
    cf = QuadraticCostFunction(2 * n, n)
    cf.Q = np.diag([Q_POS] * n + [Q_VIT] * n)
    cf.R = R_U * np.eye(n)
    return synthesize_lqr_controller(ss, cf).K


def lqr_step(x, x_ref, K, a_ref):
    u = np.clip(K @ (x_ref - x) + a_ref, -1.5 * ACC_MAX, 1.5 * ACC_MAX)
    q, dq = x[:N_JOINTS], x[N_JOINTS:]
    dq = np.clip(dq + u * DT, -1.5 * VEL_MAX, 1.5 * VEL_MAX)
    return np.concatenate([q + dq * DT, dq]), u


def draw_markers(viewer, points, rgba, size=0.005, reset=False):
    scn = viewer.user_scn
    if reset:
        scn.ngeom = 0
    for p in points:
        if scn.ngeom >= scn.maxgeom:
            break
        mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([size, 0, 0]),
                            np.asarray(p, float), np.eye(3).flatten(), np.asarray(rgba, np.float32))
        scn.ngeom += 1


# =============================================================================
# 4. PROGRAMME PRINCIPAL
# =============================================================================
def main():
    K = build_lqr()
    print("Gain LQR K (première ligne) :", np.round(K[0], 3))
    arm = None
    if MODE == "real":
        from lerobot_kinematics import feetech_arm
        arm = feetech_arm(driver_port=PORT, calibration_file=CALIB_FILE)
        q0 = np.array(arm.feedback(), dtype=float)
    else:
        q0 = Q_HOME.copy()
    planner = ArmPlanner(XML_PATH, [], table_z=0.0, verbose=False)
    segments, sommets = construire_parcours(planner, q0)
    refs = [Reference(p) for _, p in segments]
    print(f"{len(segments)} segments, durée prévue {sum(r.T for r in refs):.1f} s")
    if arm is not None:
        arm.arm_hardware.write("Goal_Position", np.rad2deg(q0) * np.array([-1, -1, 1, 1, -1, 1]))
        arm.arm_hardware.write("Torque_Enable", 1)

    m, d = planner.m, mujoco.MjData(planner.m)
    x = np.concatenate([q0, np.zeros(N_JOINTS)])
    log = {k: [] for k in ("t", "q", "ref", "u", "meas", "seg")}
    t0 = time.time()
    try:
        with mujoco.viewer.launch_passive(m, d) as viewer:
            ideal = []
            for a, b in zip(sommets[:-1], sommets[1:]):
                ideal += [a + f * (b - a) for f in np.linspace(0, 1, 25)]
            draw_markers(viewer, ideal, [1.0, 0.8, 0.0, 0.9], 0.003, reset=True)   # étoile idéale (jaune)
            for k, ((name, path), ref) in enumerate(zip(segments, refs)):
                print(f"--> {name}  ({ref.T:.1f} s)")
                t_step, t_reached = time.time(), None
                while viewer.is_running():
                    loop_start = time.time()
                    t_rel = loop_start - t_step
                    q_r, dq_r, a_r = ref(t_rel)
                    x, u = lqr_step(x, np.concatenate([q_r, dq_r]), K, a_r)
                    d.qpos[planner.qadr] = x[:N_JOINTS]
                    mujoco.mj_forward(m, d)
                    if k > 0:   # trace de la pince (vert)
                        draw_markers(viewer, [d.site_xpos[planner.site].copy()], [0.1, 0.8, 0.2, 1.0], 0.0025)
                    viewer.sync()
                    q_meas = None
                    if arm is not None:
                        arm.action(x[:N_JOINTS].copy())
                        q_meas = np.array(arm.feedback(), dtype=float)
                        if np.max(np.abs(q_meas[:5] - x[:5])) > TRACK_ERR_MAX:
                            raise RuntimeError("Écart de suivi trop grand : arrêt de sécurité")
                    log["t"].append(time.time() - t0)
                    log["q"].append(x[:N_JOINTS].copy())
                    log["ref"].append(q_r.copy())
                    log["u"].append(u.copy())
                    log["meas"].append(q_meas if q_meas is not None else np.full(N_JOINTS, np.nan))
                    log["seg"].append(k)
                    reached = (t_rel >= ref.T and np.max(np.abs(x[:N_JOINTS] - path[-1])) < POS_TOL
                               and np.max(np.abs(x[N_JOINTS:])) < 0.05)
                    if reached and t_reached is None:
                        t_reached = time.time()
                    if (t_reached is not None and time.time() - t_reached >= HOLD) or t_rel > ref.T + STEP_TIMEOUT:
                        break
                    dt_left = DT - (time.time() - loop_start)
                    if dt_left > 0:
                        time.sleep(dt_left)
                if not viewer.is_running():
                    break
        print("Parcours terminé.")
    except KeyboardInterrupt:
        print("Interrompu.")
    finally:
        if arm is not None:
            arm.disconnect()
    plot_logs(planner, sommets, log)
    return log


def plot_logs(planner, sommets, log):
    t, q, meas = np.array(log["t"]), np.array(log["q"]), np.array(log["meas"])
    seg = np.array(log["seg"])
    tip = np.array([planner.fk(qq)[0] for qq in q])
    fig = plt.figure(figsize=(12, 5))
    ax = fig.add_subplot(1, 2, 1)
    S = np.array(sommets)
    ax.plot(S[:, 1], S[:, 0], "--", color="0.6", label="étoile idéale")
    ax.plot(tip[seg > 0, 1], tip[seg > 0, 0], color="C0", label="pince (état LQR)")
    if not np.all(np.isnan(meas)):
        tipm = np.array([planner.fk(qq)[0] for qq in meas])
        ax.plot(tipm[seg > 0, 1], tipm[seg > 0, 0], color="C1", label="pince (mesure)")
    ax.set_xlabel("y (m)")
    ax.set_ylabel("x (m)")
    ax.invert_xaxis()
    ax.set_aspect("equal")
    ax.legend()
    ax.set_title(f"Parcours « {PATTERN} » vu de dessus")
    ax2 = fig.add_subplot(1, 2, 2)
    for j in range(5):
        ax2.plot(t, q[:, j], label=JOINT_NAMES[j])
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("q (rad)")
    ax2.legend(fontsize=8)
    ax2.set_title("Angles articulaires (état LQR)")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
