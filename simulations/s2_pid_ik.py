# -*- coding: utf-8 -*-
"""
Q5 : PID articulaire piloté par la cinématique inverse (partie2_so101/pid_ik_sim.py).

1. Exécute le script ORIGINAL pid_ik_sim.py tel quel (sa figure est enregistrée).
2. Analyse du modèle J q'' = τ - b q' avec PID : pôles en boucle fermée de chaque articulation,
   puis balayages de Kp, Ki, Kd (dépassement, temps de réponse).
3. Pourquoi pid_ik_real.py « ne lit pas » le bras : comparaison consigne PID / suivi du servo.
Lancer :  python simulations/s2_pid_ik.py
"""
import os
import runpy
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from outils.banc import RACINE, Capture  # noqa: E402  (ajoute le shim lerobot_kinematics)
from outils.style import plt, sauver, C, GRIS  # noqa: E402

FIG = os.path.join(RACINE, "figures", "partie2")
md = []

# --- 1. script original ----------------------------------------------------------------------
print("=== pid_ik_sim.py (original) ===")
cap = Capture(FIG, "p2_q5_pid_ik_sim_original")
plt.show = cap
with plt.style.context("default"):
    runpy.run_path(os.path.join(RACINE, "partie2_so101", "pid_ik_sim.py"), run_name="__main__")

print("=== pid_ik_sim.py avec le correctif ik_precise (tolérance 1e-10) ===")
import lerobot_kinematics  # noqa: E402
sys.path.insert(0, os.path.join(RACINE, "partie2_so101"))
from ik_precise import lerobot_IK_precis  # noqa: E402
_orig_ik = lerobot_kinematics.lerobot_IK
lerobot_kinematics.lerobot_IK = lambda q, g, robot: lerobot_IK_precis(q, g, robot, pas_max=None)
plt.show = Capture(FIG, "p2_q5_pid_ik_sim_correctif")
with plt.style.context("default"):
    runpy.run_path(os.path.join(RACINE, "partie2_so101", "pid_ik_sim.py"), run_name="__main__")
lerobot_kinematics.lerobot_IK = _orig_ik

# --- 2. analyse du PID sur le modèle par articulation ----------------------------------------
J = np.array([0.05, 0.04, 0.02, 0.01])
b = np.array([0.20, 0.15, 0.08, 0.04])
kp, ki, kd = 2.0, 0.5, 0.15
noms = ["Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]
md.append("| Articulation | J | b | pôles en boucle fermée J s³ + (b+Kd) s² + Kp s + Ki = 0 | pôle dominant |")
md.append("|---|---|---|---|---|")
for k in range(4):
    p = np.roots([J[k], b[k] + kd, kp, ki])
    dom = p[np.argmax(p.real)]
    md.append(f"| {noms[k]} | {J[k]} | {b[k]} | {', '.join(f'{z.real:.2f}{z.imag:+.2f}j' for z in p)} | "
              f"{dom.real:.2f} (τ ≈ {-1 / dom.real:.1f} s) |")


def reponse(Jk, bk, kp, ki, kd, T=4.0, dt=0.005, sat=1.5, i_lim=0.5):
    """Réponse indicielle (consigne 0.1 rad) du PID discret de pid_ik_sim.py."""
    q, dq, integ, e_prev = 0.0, 0.0, 0.0, None
    out = []
    for _ in range(int(T / dt)):
        e = 0.1 - q
        integ = np.clip(integ + e * dt, -i_lim, i_lim)
        der = 0.0 if e_prev is None else (e - e_prev) / dt
        e_prev = e
        tau = np.clip(kp * e + ki * integ + kd * der, -sat, sat)
        dq += (tau - bk * dq) / Jk * dt
        q += dq * dt
        out.append(q)
    return np.arange(len(out)) * dt, np.array(out) / 0.1


def indicateurs(t, y):
    d = max(0.0, y.max() - 1) * 100
    hors = np.where(np.abs(y - 1) > 0.02)[0]
    return d, (t[hors[-1]] if len(hors) else 0.0)


fig, ax = plt.subplots(1, 3, figsize=(15, 4.2), sharey=True)
for col, (nom, vals) in enumerate([("Kp", [0.5, 1, 2, 5, 10]), ("Ki", [0, 0.5, 2, 5, 10]),
                                   ("Kd", [0, 0.05, 0.15, 0.5, 1.5])]):
    for i, v in enumerate(vals):
        g = dict(kp=kp, ki=ki, kd=kd)
        g[nom.lower()] = v
        t, y = reponse(J[0], b[0], **g)
        dep, ts = indicateurs(t, y)
        ax[col].plot(t, y, color=C[i], label=f"{nom} = {v:g}  (D {dep:.0f} %, t2% {ts:.1f} s)")
        md.append(f"| Pitch, {nom} = {v:g} | dépassement {dep:.0f} % | temps de réponse à 2 % {ts:.2f} s |")
    ax[col].axhline(1, color=GRIS, ls="--", lw=1)
    ax[col].set_title(f"Effet de {nom} (articulation Pitch, J = 0.05, b = 0.2)")
    ax[col].set_xlabel("t (s)")
    ax[col].legend(fontsize=7.5, loc="lower right")
ax[0].set_ylabel("q / consigne")
fig.suptitle("Q5 — PID de pid_ik_sim.py : influence de chaque gain (référence Kp=2, Ki=0.5, Kd=0.15)",
             fontweight="bold")
sauver(fig, "partie2", "p2_q5_pid_balayages")

with open(os.path.join(FIG, "p2_q5_pid_metriques.md"), "w") as f:
    f.write("\n".join(md[:6]) + "\n\n| Essai | Dépassement | Temps de réponse |\n|---|---|---|\n")
    f.write("\n".join(l for l in md[6:]) + "\n")
print("\n".join(md))
