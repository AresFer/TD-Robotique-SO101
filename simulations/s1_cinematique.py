# -*- coding: utf-8 -*-
"""
Q4 / Q5 : cinématique directe et inverse du SO-101 (lerobot-kinematics) — vérifications.

1. Le modèle géométrique de lerobot-kinematics (so101) est-il cohérent avec la scène MuJoCo ?
   On compare lerobot_FK(q) au repère du poignet MuJoCo pour 500 configurations aléatoires.
2. Comportement de lerobot_IK : un appel ne bouge chaque articulation que de 0.1 rad au plus
   (smooth_joint_motion) -> combien d'appels pour atteindre une cible ? Taux de réussite de
   la stratégie « solve_ik » de nos scripts (rapprochement progressif de la cible).
3. Espace de travail atteignable dans le plan vertical du bras.
Lancer :  python simulations/s1_cinematique.py
"""
import contextlib
import io
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from outils.banc import RACINE, espace_travail, EX  # noqa: E402  (ajoute aussi le shim au path)
from outils.style import plt, sauver, C, GRIS  # noqa: E402
import mujoco  # noqa: E402
from lerobot_kinematics import get_robot, lerobot_FK, lerobot_IK  # noqa: E402
sys.path.insert(0, os.path.join(RACINE, "partie2_so101"))
from ik_precise import lerobot_IK_precis  # noqa: E402

espace_travail()
rob = get_robot("so101")
m = mujoco.MjModel.from_xml_path(os.path.join(EX, "scene_so101.xml"))
d = mujoco.MjData(m)
NOMS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
idx = [m.jnt_qposadr[m.joint(n).id] for n in NOMS]
rng = np.random.default_rng(1)
md = []

# --- 1. cohérence FK lerobot / MuJoCo ------------------------------------------------------
E, Pm = [], []
for _ in range(500):
    q = np.zeros(6)
    q[1:4] = rng.uniform([-1.5, -1.5, -1.5], [1.5, 1.5, 1.5])
    d.qpos[idx] = q
    mujoco.mj_kinematics(m, d)
    rel = d.xpos[m.body("gripper").id] - d.xpos[m.body("shoulder").id]
    g = lerobot_FK(q[1:5], rob)
    E.append([rel[0] - g[0], rel[2] - g[2]])
    Pm.append(rel)
E = 1000 * np.array(E)
fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
ax[0].hist(E[:, 0], bins=30, color=C[0], alpha=0.85, label="écart en x")
ax[0].hist(E[:, 1], bins=30, color=C[1], alpha=0.65, label="écart en z")
ax[0].set_xlabel("FK MuJoCo − FK lerobot-kinematics (mm)")
ax[0].set_ylabel("nombre de configurations")
ax[0].set_title("Cohérence des deux modèles (repère du poignet, 500 tirages)")
ax[0].legend()
norme = np.linalg.norm(E, axis=1)
ax[1].scatter(np.array(Pm)[:, 0], np.array(Pm)[:, 2], c=norme, cmap="viridis", s=10)
sc = ax[1].collections[0]
fig.colorbar(sc, ax=ax[1], label="écart (mm)")
ax[1].set_xlabel("x poignet / épaule (m)")
ax[1].set_ylabel("z poignet / épaule (m)")
ax[1].set_aspect("equal")
ax[1].set_title("Où l'écart est-il le plus grand ?")
sauver(fig, "partie2", "p2_q5_fk_lerobot_vs_mujoco")
md.append(f"| Écart FK moyen (x, z) | {E[:, 0].mean():+.1f} mm, {E[:, 1].mean():+.1f} mm |")
md.append(f"| Écart FK : écart-type (x, z) | {E[:, 0].std():.1f} mm, {E[:, 1].std():.1f} mm |")
md.append(f"| Écart FK maximal | {norme.max():.1f} mm |")

# --- 2. lerobot_IK : nombre d'appels et stratégie solve_ik ---------------------------------
q_home = np.zeros(4)
g_home = lerobot_FK(q_home, rob)


def solve_ik(gpos_target, q4_seed, n_interp=40, n_iter=15):
    """Copie de solve_ik() de pick_place_trajectoire_LQR.py."""
    q4 = np.array(q4_seed, dtype=float)
    g_start = lerobot_FK(q4, robot=rob)
    for k in range(1, n_interp + 1):
        g_k = g_start + (gpos_target - g_start) * k / n_interp
        for _ in range(n_iter):
            with contextlib.redirect_stdout(io.StringIO()):
                q_new, ok = lerobot_IK(q4, g_k, robot=rob)
            if not ok:
                break
            done = np.max(np.abs(q_new - q4)) < 1e-4
            q4 = np.array(q_new, dtype=float)
            if done:
                break
    return q4, np.linalg.norm(lerobot_FK(q4, robot=rob)[:3] - gpos_target[:3])


res_direct, res_solve, res_fix, appels, durees = [], [], [], [], []
cibles = []
for _ in range(60):
    q_t = rng.uniform([-1.2, -1.2, -1.2, 0], [1.2, 1.2, 1.2, 0])
    g = lerobot_FK(q_t, rob)                  # cible atteignable par construction (pose complète)
    cibles.append(g)
    # (a) appels répétés de lerobot_IK depuis la pose de repos
    q, n, err = q_home.copy(), 0, np.inf
    with contextlib.redirect_stdout(io.StringIO()):
        for n in range(1, 61):
            q_new, ok = lerobot_IK(q, g, robot=rob)
            if not ok:
                break
            q = np.array(q_new, float)
            err = np.linalg.norm(lerobot_FK(q, rob)[:3] - g[:3])
            if err < 5e-3:
                break
    res_direct.append(err)
    appels.append(n)
    t0 = time.time()
    _, e2 = solve_ik(g, q_home)
    durees.append(time.time() - t0)
    res_solve.append(e2)
    qf, okf = lerobot_IK_precis(q_home, g, rob)
    res_fix.append(np.linalg.norm(lerobot_FK(qf, rob)[:3] - g[:3]) if okf else 1.0)
res_direct, res_solve, res_fix = 1000 * np.array(res_direct), 1000 * np.array(res_solve), 1000 * np.array(res_fix)
fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
ordre = np.argsort(res_solve)
ax[0].semilogy(res_direct[ordre], "o", color=C[1], ms=4, label="lerobot_IK répété (≤ 60 appels)")
ax[0].semilogy(res_solve[ordre], "o", color=C[0], ms=4, label="solve_ik (cible approchée en 40 pas)")
ax[0].semilogy(np.clip(res_fix[ordre], 1e-4, None), "o", color=C[2], ms=4,
               label="correctif ik_precise (tol 1e-10, 1 appel)")
ax[0].axhline(5, color=GRIS, ls="--", lw=1)
ax[0].axhline(44.7, color=C[7], ls=":", lw=1)
ax[0].text(1, 50, "√(2·tol) = 45 mm : seuil d'arrêt réel de lerobot_IK", color=C[7], fontsize=8)
ax[0].text(1, 5.6, "seuil 5 mm de nos scripts", color=GRIS, fontsize=8)
ax[0].set_xlabel("cible n° (triées)")
ax[0].set_ylabel("erreur finale de position (mm)")
ax[0].set_title("Précision de l'IK sur 60 cibles atteignables (q tirés au hasard)")
ax[0].legend(fontsize=8)
ax[1].hist(appels, bins=np.arange(0, 62, 3), color=C[2])
ax[1].set_xlabel("nombre d'appels à lerobot_IK pour converger (< 5 mm)")
ax[1].set_ylabel("nombre de cibles")
ax[1].set_title("Chaque appel est limité à 0.1 rad par articulation\n(60 = pas de convergence)")
sauver(fig, "partie2", "p2_q5_ik_lerobot")
md.append(f"| IK : réussite (< 5 mm) avec lerobot_IK répété | {np.mean(res_direct < 5) * 100:.0f} % |")
md.append(f"| IK : réussite (< 5 mm) avec solve_ik | {np.mean(res_solve < 5) * 100:.0f} % |")
md.append(f"| IK : réussite (< 5 mm) avec le correctif ik_precise (1 appel) | {np.mean(res_fix < 5) * 100:.0f} % (médiane {np.median(res_fix):.4f} mm) |")
md.append(f"| IK : appels médians à lerobot_IK | {int(np.median(appels))} |")
md.append(f"| IK : durée moyenne de solve_ik | {1000 * np.mean(durees):.0f} ms |")

# --- 3. espace de travail dans le plan vertical ----------------------------------------------
Q = rng.uniform(rob.qlim[0], rob.qlim[1], size=(6000, 4))
Q[:, 3] = 0
P = np.array([lerobot_FK(q, rob)[:3] for q in Q])
fig, a = plt.subplots(figsize=(6.5, 5))
a.scatter(P[:, 0], P[:, 2], s=2, color=C[0], alpha=0.35)
a.plot(0, 0, "s", color=C[7], ms=8, label="axe de l'épaule (origine du modèle)")
a.plot(g_home[0], g_home[2], "*", color=C[1], ms=14, label="q = 0")
a.set_aspect("equal")
a.set_xlabel("x horizontal (m)")
a.set_ylabel("z (m)")
a.set_title("Espace atteignable du poignet (butées qlim de lerobot-kinematics)")
a.legend(loc="lower left", fontsize=8)
sauver(fig, "partie2", "p2_q4_espace_travail")

with open(os.path.join(RACINE, "figures", "partie2", "p2_q4_q5_cinematique.md"), "w") as f:
    f.write("| Vérification | Résultat |\n|---|---|\n" + "\n".join(md) + "\n")
print("\n".join(md))
