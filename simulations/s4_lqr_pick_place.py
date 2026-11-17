# -*- coding: utf-8 -*-
"""
Q6 / Q7 : LQR (pyro) + pick & place — script ORIGINAL partie2_so101/pick_place_trajectoire_LQR.py

1. Exécution telle quelle : build_waypoints échoue (IK d'origine, tolérance 1e-3 -> 45 mm).
2. Exécution avec le correctif ik_precise (même script, lerobot_IK remplacé) :
   simulation pure (réponse indicielle pyro + journal) et jumeau numérique (+ GIF).
3. Théorie du LQR « double intégrateur » par articulation : K, ωn, ζ en fonction de Q et R.
4. Balayage de Q (poids position) sur le jumeau : suivi de consigne, effort, durée.
Lancer :  python simulations/s4_lqr_pick_place.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from outils.banc import executer, effacer_points, RACINE  # noqa: E402
from outils.style import plt, sauver, C, GRIS  # noqa: E402

sys.path.insert(0, os.path.join(RACINE, "partie2_so101"))
from ik_precise import lerobot_IK_precis  # noqa: E402

FIG = os.path.join(RACINE, "figures", "partie2")
GIF = os.path.join(RACINE, "figures", "gif")
CORRECTIF = dict(lerobot_IK=lambda q, g, robot: lerobot_IK_precis(q, g, robot, pas_max=0.1))
md = []
effacer_points()

# --- 1. tel quel ------------------------------------------------------------------------------
print("=== 1. script original, IK d'origine ===")
try:
    executer("pick_place_trajectoire_LQR", "sim", "", None, reglages=dict(SHOW_PYRO_STEP=False),
             kw_vue=dict(actif=False))
    md.append("- Script original + lerobot_IK d'origine : séquence exécutée")
except ValueError as e:
    print("   ->", e)
    md.append(f"- Script original + lerobot_IK d'origine : **échec** — `{e}`")

# --- 2. avec correctif --------------------------------------------------------------------------
print("=== 2. correctif ik_precise : simulation pure ===")
executer("pick_place_trajectoire_LQR", "sim", "p2_q7_lqr_v1_sim", FIG,
         reglages=dict(SHOW_PYRO_STEP=True, **CORRECTIF), kw_vue=dict(actif=False))
print("=== 2bis. correctif ik_precise : jumeau numérique ===")
mod, logs = executer("pick_place_trajectoire_LQR", "real", "p2_q7_lqr_v1_jumeau", FIG,
                     gif=os.path.join(GIF, "q7_pick_place_lqr_jumeau.gif"), q0_jumeau=np.zeros(6),
                     reglages=dict(SHOW_PYRO_STEP=False, **CORRECTIF),
                     kw_vue=dict(tous_les=4, largeur=420, hauteur=315))
e = np.abs(logs["meas"][:, :5] - logs["q"][:, :5])
md.append(f"- Correctif, jumeau : durée {logs['t'][-1]:.1f} s ; erreur mesurée max par articulation "
          f"{np.round(e.max(0), 3).tolist()} rad ; erreur statique d'épaule (gravité) ≈ "
          f"{np.median(logs['meas'][:, 1] - logs['q'][:, 1]):+.3f} rad")

# --- 3. théorie : LQR d'un double intégrateur ----------------------------------------------------
from pyro.dynamic.statespace import StateSpaceSystem  # noqa: E402
from pyro.analysis.costfunction import QuadraticCostFunction  # noqa: E402
from pyro.control.lqr import synthesize_lqr_controller  # noqa: E402


def K1(qp, qv, r):
    ss = StateSpaceSystem(np.array([[0, 1], [0, 0.]]), np.array([[0], [1.]]), np.eye(2), np.zeros((2, 1)))
    cf = QuadraticCostFunction(2, 1)
    cf.Q, cf.R = np.diag([qp, qv]), np.array([[r]])
    return synthesize_lqr_controller(ss, cf).K[0]


fig, ax = plt.subplots(1, 3, figsize=(16, 4.3))
t = np.linspace(0, 3, 601)
tab = ["| Q position | Q vitesse | R | kp = √(Q/R) | kd | ωn (rad/s) | ζ | temps de réponse 5 % |",
       "|---|---|---|---|---|---|---|---|"]
for i, (qp, qv, r) in enumerate([(1, 1, 1), (9, 1, 1), (81, 1, 1), (729, 1, 1), (81, 30, 1)]):
    kp, kd = K1(qp, qv, r)
    A = np.array([[0, 1], [-kp, -kd]])
    x, X = np.array([-1.0, 0]), []
    for _ in t:
        X.append(x[0])
        x = x + 0.005 * A @ x
    X = 1 + np.array(X)
    wn, z = np.sqrt(kp), kd / (2 * np.sqrt(kp))
    hors = np.where(np.abs(X - 1) > 0.05)[0]
    t5 = t[hors[-1]] if len(hors) else 0
    ax[0].plot(t, X, color=C[i], label=f"Q=({qp:g},{qv:g}) R={r:g} : K=[{kp:.2f}, {kd:.2f}]")
    tab.append(f"| {qp:g} | {qv:g} | {r:g} | {kp:.2f} | {kd:.3f} | {wn:.2f} | {z:.3f} | {t5:.2f} s |")
ax[0].set_title("Réponse à un échelon de consigne (sans loi horaire)")
ax[0].set_xlabel("t (s)")
ax[0].set_ylabel("q / consigne")
ax[0].legend(fontsize=7.5)
qq = np.geomspace(0.1, 1e4, 200)
ax[1].semilogx(qq, [K1(q, 1, 1)[0] for q in qq], color=C[0], label="kp")
ax[1].semilogx(qq, [K1(q, 1, 1)[1] for q in qq], color=C[1], label="kd")
ax[1].axvline(81, color=GRIS, ls=":", lw=1)
ax[1].text(90, 1, "nos scripts : Q = 81", color=GRIS, fontsize=8)
ax[1].set_xlabel("Q position (R = 1, Q vitesse = 1)")
ax[1].set_title("kp = √Q, kd = √(1 + 2√Q)")
ax[1].legend()
ax[2].semilogx(qq, [K1(q, 1, 1)[1] / (2 * np.sqrt(K1(q, 1, 1)[0])) for q in qq], color=C[2])
ax[2].axhline(1 / np.sqrt(2), color=GRIS, ls="--", lw=1)
ax[2].text(0.15, 0.73, "ζ → 1/√2 quand Q augmente", color=GRIS, fontsize=8)
ax[2].set_xlabel("Q position")
ax[2].set_ylabel("ζ")
ax[2].set_title("Amortissement en boucle fermée")
fig.suptitle("Q6 — LQR d'une articulation modélisée en double intégrateur q'' = u (pyro)", fontweight="bold")
sauver(fig, "partie2", "p2_q6_lqr_double_integrateur")

# --- 4. balayage de Q sur le jumeau ----------------------------------------------------------
print("=== 4. balayage de Q sur le jumeau ===")
tab2 = ["| Q position | durée (s) | max\\|x − consigne\\| (rad) | max\\|mesure − x\\| (rad) | max\\|u\\| (rad/s²) |",
        "|---|---|---|---|---|"]
fig, ax = plt.subplots(1, 2, figsize=(13, 4.2))
for i, qp in enumerate([1, 9, 81, 729]):
    Q = np.diag([qp] * 6 + [1.0] * 6)
    mod, lg = executer("pick_place_trajectoire_LQR", "real", "", None, q0_jumeau=np.zeros(6),
                       reglages=dict(SHOW_PYRO_STEP=False, Q=Q, **CORRECTIF), kw_vue=dict(actif=False))
    suivi = np.abs(lg["q"][:, :5] - lg["ref"][:, :5]).max()
    mes = np.abs(lg["meas"][:, :5] - lg["q"][:, :5]).max()
    tab2.append(f"| {qp:g} | {lg['t'][-1]:.1f} | {suivi:.3f} | {mes:.3f} | {np.abs(lg['u']).max():.1f} |")
    sel = (lg["t"] > 2.0) & (lg["t"] < 4.5)
    ax[0].plot(lg["t"][sel], lg["q"][sel, 0], color=C[i], label=f"Q = {qp:g}")
    ax[1].plot(lg["t"][sel], lg["u"][sel, 0], color=C[i], lw=1.2, label=f"Q = {qp:g}")
    if i == 0:
        ax[0].plot(lg["t"][sel], lg["ref"][sel, 0], "--", color=GRIS, label="consigne (loi horaire)")
ax[0].set_title("shoulder_pan pendant « Au-dessus de A » : état LQR")
ax[0].set_xlabel("t (s)")
ax[0].set_ylabel("rad")
ax[0].legend(fontsize=8)
ax[1].set_title("Commande u = q'' : avec l'anticipation ddq_ref, Q change peu u")
ax[1].set_xlabel("t (s)")
ax[1].set_ylabel("rad/s²")
sauver(fig, "partie2", "p2_q7_balayage_Q_jumeau")

with open(os.path.join(FIG, "p2_q6_q7_lqr.md"), "w") as f:
    f.write("\n".join(md) + "\n\n" + "\n".join(tab) + "\n\n" + "\n".join(tab2) + "\n")
print("\n".join(md + [""] + tab + [""] + tab2))
