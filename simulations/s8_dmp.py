# -*- coding: utf-8 -*-
"""
Aller plus loin : DMP (Dynamic Movement Primitives) — partie2_so101/dmp.py

1. DMP cartésienne apprise sur le chemin de la pince planifié par A* (contournement du poteau),
   puis : reproduction, nouveaux buts, accélération / ralentissement, nombre de gaussiennes.
2. Couplage d'évitement : une ligne droite démontrée contourne un obstacle ajouté après coup.
3. DMP articulaire exécutée sur le jumeau numérique vers un nouveau but + vérification des
   collisions (la DMP généralise la forme, PAS la sécurité).
Lancer :  python simulations/s8_dmp.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from outils.banc import charger, effacer_points, RACINE, EX  # noqa: E402
from outils.style import plt, sauver, C, GRIS  # noqa: E402
from outils.jumeau import BrasFeetechSimule  # noqa: E402

effacer_points()
mod = charger("pick_place_astar")
import astar_planner  # noqa: E402
from dmp import DMP  # noqa: E402

P = astar_planner.ArmPlanner(mod.XML_PATH, mod.OBSTACLES, table_z=0.02, verbose=False)
qA, qB = mod.load_points(P)
qa, qb = mod.above(P, qA, 0.06), mod.above(P, qB, 0.06)
qa[5] = qb[5] = 0.0
path, _ = P.plan(qa, qb)
tip = np.array([P.fk(q)[0] for q in path])
# ré-échantillonnage uniforme en abscisse curviligne, durée 3 s
s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(tip, axis=0), axis=1))])
N = 151
su = np.linspace(0, s[-1], N)
u = np.linspace(0, 1, N)
su = s[-1] * (3 * u ** 2 - 2 * u ** 3)                    # départ / arrivée à vitesse nulle
demo = np.array([np.interp(su, s, tip[:, k]) for k in range(3)]).T
dt = 3.0 / (N - 1)
md = []

# --- 1. reproduction / nouveaux buts / vitesse ------------------------------------------------
dmp = DMP(3, n_bases=30).apprendre(demo, dt)
rep = dmp.rejouer(dt=dt)
err = np.linalg.norm(rep[:N] - demo, axis=1)
md.append(f"| Reproduction (30 gaussiennes) | écart moyen {1000 * err.mean():.2f} mm, max {1000 * err.max():.2f} mm |")
o = mod.OBSTACLES[0]
c, h = np.array(o["centre"]), np.array(o["demi_taille"])
fig, ax = plt.subplots(1, 3, figsize=(16, 5))
a = ax[0]
a.add_patch(plt.Rectangle((c[1] - h[1], c[0] - h[0]), 2 * h[1], 2 * h[0], color=C[7], alpha=0.5))
a.plot(demo[:, 1], demo[:, 0], "--", color=GRIS, lw=2.5, label="démonstration (A*)")
a.plot(rep[:, 1], rep[:, 0], color=C[0], label="DMP, même but")
for i, dg in enumerate([[0.0, 0.06, 0], [0.04, -0.04, 0], [-0.04, 0.03, 0]]):
    g2 = dmp.g + np.array(dg)
    r2 = dmp.rejouer(g=g2, dt=dt)
    a.plot(r2[:, 1], r2[:, 0], color=C[i + 1], label=f"nouveau but Δ = {np.round(dg[:2], 2)}")
    a.plot(g2[1], g2[0], "o", color=C[i + 1])
a.invert_xaxis()
a.set_aspect("equal")
a.set_xlabel("y (m)")
a.set_ylabel("x (m)")
a.set_title("Généralisation à de nouveaux buts (vue de dessus)")
a.legend(fontsize=7.5, loc="lower left")
a = ax[1]
for i, v in enumerate([0.5, 1.0, 2.0]):
    r = dmp.rejouer(tau=dmp.tau / v, dt=0.005)
    vit = np.linalg.norm(np.gradient(r, 0.005, axis=0), axis=1)
    a.plot(np.arange(len(r)) * 0.005, vit, color=C[i], label=f"vitesse × {v:g} (τ = {dmp.tau / v:.1f} s)")
a.set_xlabel("t (s)")
a.set_ylabel("vitesse de la pince (m/s)")
a.set_title("Même forme, durée réglée par τ")
a.legend(fontsize=8)
a = ax[2]
nbs = [3, 5, 10, 20, 30, 60]
errs = []
for nb in nbs:
    r = DMP(3, n_bases=nb).apprendre(demo, dt).rejouer(dt=dt)
    errs.append(1000 * np.linalg.norm(r[:N] - demo, axis=1).mean())
    md.append(f"| {nb} gaussiennes | écart moyen à la démonstration {errs[-1]:.2f} mm |")
a.semilogy(nbs, errs, "o-", color=C[0])
a.set_xlabel("nombre de gaussiennes")
a.set_ylabel("écart moyen à la démonstration (mm)")
a.set_title("Richesse du terme de forçage")
fig.suptitle("DMP cartésienne apprise sur le chemin A* de la pince", fontweight="bold")
sauver(fig, "bonus", "b2_dmp_generalisation")

# --- 2. couplage d'évitement ----------------------------------------------------------------
y0, g = np.array([0.20, -0.12, 0.10]), np.array([0.20, 0.14, 0.10])
t = np.linspace(0, 1, N)
ligne = y0 + (g - y0) * (3 * t ** 2 - 2 * t ** 3)[:, None]
d2 = DMP(3).apprendre(ligne, dt)
obs = np.array([0.205, 0.01, 0.10])        # légèrement hors de la ligne (θ = 0 annule le couplage)
fig, a = plt.subplots(figsize=(7, 4.5))
a.plot(ligne[:, 1], ligne[:, 0], "--", color=GRIS, label="démonstration (ligne droite)")
for i, gam in enumerate([0, 400, 1000, 2000]):
    r = d2.rejouer(dt=dt, obstacles=[obs], gamma=gam, beta=3.0)
    dmin = np.min(np.linalg.norm(r - obs, axis=1))
    a.plot(r[:, 1], r[:, 0], color=C[i], label=f"γ = {gam}  (distance min {100 * dmin:.1f} cm)")
    md.append(f"| Évitement γ = {gam} | distance minimale à l'obstacle {100 * dmin:.1f} cm ; erreur finale {1000 * np.linalg.norm(r[-1] - g):.1f} mm |")
a.plot(obs[1], obs[0], "o", color=C[7], ms=14, label="obstacle ponctuel")
a.invert_xaxis()
a.set_aspect("equal")
a.set_xlabel("y (m)")
a.set_ylabel("x (m)")
a.legend(fontsize=8)
a.set_title("DMP + couplage d'évitement (Hoffmann 2009)")
sauver(fig, "bonus", "b2_dmp_evitement")

# --- 3. DMP articulaire sur le jumeau, nouveau but ---------------------------------------------
dq = DMP(4).apprendre(path[:, :4], 3.0 / (len(path) - 1))
fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
ax[0].add_patch(plt.Rectangle((c[1] - h[1], c[0] - h[0]), 2 * h[1], 2 * h[0], color=C[7], alpha=0.5))
for i, dpan in enumerate([0.0, -0.25, 0.25]):
    g4 = dq.g + np.array([dpan, 0, 0, 0])
    Q4 = dq.rejouer(g=g4, dt=0.02)
    Q6 = np.hstack([Q4, np.tile(path[0, 4:], (len(Q4), 1))])
    coll = sum(P.collides(q) for q in Q6)
    bras = BrasFeetechSimule(os.path.join(EX, "scene_so101.xml"), q0=Q6[0], dt_boucle=0.02)
    mes = np.array([bras.action(q) for q in Q6])
    tp = np.array([P.fk(q)[0] for q in Q6])
    tm = np.array([P.fk(q)[0] for q in mes])
    ax[0].plot(tp[:, 1], tp[:, 0], color=C[i], label=f"but pan {dpan:+.2f} rad : {coll} config. en collision")
    ax[0].plot(tm[:, 1], tm[:, 0], ":", color=C[i])
    ax[1].plot(np.arange(len(Q4)) * 0.02, Q4[:, 0], color=C[i], label=f"pan, but {dpan:+.2f}")
    md.append(f"| DMP articulaire, but pan {dpan:+.2f} rad | {coll} configurations en collision sur {len(Q6)} ; "
              f"écart pince consigne/mesure jumeau {1000 * np.linalg.norm(tp - tm, axis=1).mean():.1f} mm |")
ax[0].invert_xaxis()
ax[0].set_aspect("equal")
ax[0].set_xlabel("y (m)")
ax[0].set_ylabel("x (m)")
ax[0].set_title("Pince (— consigne DMP, ··· mesure jumeau)")
ax[0].legend(fontsize=7.5, loc="lower left")
ax[1].set_xlabel("t (s)")
ax[1].set_ylabel("rad")
ax[1].set_title("Rotation de la base générée par la DMP")
ax[1].legend(fontsize=8)
fig.suptitle("DMP articulaire exécutée sur le jumeau : elle généralise la forme, pas l'évitement",
             fontweight="bold")
sauver(fig, "bonus", "b2_dmp_jumeau")

with open(os.path.join(RACINE, "figures", "bonus", "b2_dmp.md"), "w") as f:
    f.write("| Essai | Résultat |\n|---|---|\n" + "\n".join(md) + "\n")
print("\n".join(md))
