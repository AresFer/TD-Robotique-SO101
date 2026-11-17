# -*- coding: utf-8 -*-
"""
Aller plus loin — robustesse PID vs LQR sur le pendule EDUKIT.

1. Matrice de robustesse : bruit capteur, retard, erreur de modèle, saturation, calage.
2. Retard maximal tolérable (balayage 0 -> 40 ms).
3. Bassin d'attraction : plus grand angle initial rattrapé.
4. Marges de stabilité (lieu de Nyquist de la boucle ouverte à l'entrée du moteur).
Lancer :  python partie1_pendule/robustesse_pendule.py
"""
import os
import sys

import numpy as np

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [ICI, os.path.dirname(ICI)]
from outils.style import plt, sauver, C, GRIS  # noqa: E402
from commandes import (PIDShawn, DoublePID, LQR, LQI, gain_lqr, gain_lqi,  # noqa: E402
                       simuler, metriques, DEG)
from modele_pendule import PARAM_EDUKIT, matrices_lineaires  # noqa: E402

OUT = os.path.join(os.path.dirname(ICI), "figures", "partie1")
Q0, R0 = np.diag([100.0, 100.0, 1.0, 1.0]), 1.0
K0 = gain_lqr(Q0, R0)
K_lqi = gain_lqi(Q0, R0, 100.0)
CTRL = [("PID Shawn", lambda: PIDShawn(Kp=0.7, Ki=0.005, Ts=0.04)),
        ("Double PID", lambda: DoublePID()),
        ("LQR", lambda: LQR(K0)),
        ("LQI", lambda: LQI(K_lqi))]
p48 = dict(PARAM_EDUKIT, wg2=48.0, wr2=48.0 * 0.14 / 9.81)
SCEN = [("nominal", {}),
        ("bruit 0.3°", dict(bruit=0.3 * DEG)),
        ("retard 10 ms", dict(retard=0.010)),
        ("retard 25 ms", dict(retard=0.025)),
        ("ωg² réel = 48", dict(p=p48)),
        ("u max 15 rad/s²", dict(u_max=15.0)),
        ("calage 1°", dict(calage=1.0 * DEG))]
base = dict(T=8, x0=(0, 2 * DEG, 0, 0), poussee=(2.0, 0.5))

# --- 1. matrice ----------------------------------------------------------------------------
print("matrice de robustesse ...")
M = np.full((len(CTRL), len(SCEN)), np.nan)
TXT = [["" for _ in SCEN] for _ in CTRL]
for i, (nc, fab) in enumerate(CTRL):
    for j, (ns, kw) in enumerate(SCEN):
        lg = simuler(fab(), **base, **kw)
        m = metriques(lg)
        if lg["tombe"]:
            TXT[i][j] = "tombe" if lg["raison"] == "pendule tombé" else "butée\nrotor"
        else:
            M[i, j] = m["rms_alpha"]
            TXT[i][j] = f"{m['rms_alpha']:.2f}°\nθmax {m['theta_max']:.0f}°"
fig, a = plt.subplots(figsize=(13, 3.9))
cm = plt.get_cmap("Blues").copy()
cm.set_bad("#f2d4d3")
a.imshow(np.ma.masked_invalid(M), cmap=cm, vmin=0, vmax=2.0, aspect="auto")
for i in range(len(CTRL)):
    for j in range(len(SCEN)):
        a.text(j, i, TXT[i][j], ha="center", va="center", fontsize=8.5,
               color="#7a1f1e" if np.isnan(M[i, j]) else ("white" if M[i, j] > 1.2 else "#0b0b0b"))
a.set_xticks(range(len(SCEN)), [s[0] for s in SCEN])
a.set_yticks(range(len(CTRL)), [c[0] for c in CTRL])
a.grid(False)
a.set_title("Robustesse : RMS de α (bleu = bon) et rotation max du rotor ; rouge = échec "
            "(départ 2°, poussée à 2 s)")
sauver(fig, "partie1", "p1_bonus_matrice_robustesse")

# --- 2. retard maximal -----------------------------------------------------------------------
print("balayage du retard ...")
retards = np.arange(0, 0.0801, 0.005)
fig, a = plt.subplots(figsize=(8, 4.4))
lim = {}
for i, (nc, fab) in enumerate(CTRL[1:], start=1):
    rms = []
    for r in retards:
        lg = simuler(fab(), **base, retard=r)
        rms.append(np.nan if lg["tombe"] else metriques(lg)["rms_alpha"])
    rms = np.array(rms)
    a.plot(retards * 1000, rms, "o-", color=C[i], ms=4, label=nc)
    ok = retards[~np.isnan(rms)]
    lim[nc] = ok.max() * 1000 if len(ok) else 0
a.set_xlabel("retard pur dans la boucle (ms)")
a.set_ylabel("RMS α (°) — courbe interrompue = chute")
a.set_title("Retard tolérable (le firmware EDUKIT tourne à 2 ms)")
a.legend()
sauver(fig, "partie1", "p1_bonus_retard")
print("  retard max toléré :", lim)

# --- 3. bassin d'attraction ----------------------------------------------------------------
print("bassin d'attraction ...")
res_bassin = {}
for nc, fab in CTRL:
    lo, hi = 0.0, 80.0
    for _ in range(12):
        mid = (lo + hi) / 2
        lg = simuler(fab(), T=4, x0=(0, mid * DEG, 0, 0), calage=0.0, garde=85 * DEG)
        lo, hi = (mid, hi) if not lg["tombe"] else (lo, mid)
    res_bassin[nc] = lo
Rs = [0.01, 0.1, 1, 10, 100]
bassin_R = []
for r in Rs:
    lo, hi = 0.0, 80.0
    for _ in range(12):
        mid = (lo + hi) / 2
        lg = simuler(LQR(gain_lqr(Q0, r, avec_pyro=False)), T=4, x0=(0, mid * DEG, 0, 0), garde=85 * DEG)
        lo, hi = (mid, hi) if not lg["tombe"] else (lo, mid)
    bassin_R.append(lo)
fig, ax = plt.subplots(1, 2, figsize=(12, 4))
noms = list(res_bassin)
ax[0].barh(noms, [res_bassin[n] for n in noms], color=[C[i] for i in range(len(noms))], height=0.55)
for i, n in enumerate(noms):
    ax[0].text(res_bassin[n] + 0.2, i, f"{res_bassin[n]:.1f}°", va="center", fontsize=9)
ax[0].set_xlabel("angle initial maximal rattrapé (°)")
ax[0].set_title("Bassin d'attraction (accélération ≤ 60 rad/s²)")
ax[0].invert_yaxis()
ax[1].semilogx(Rs, bassin_R, "o-", color=C[2])
ax[1].set_xlabel("R (Q fixé)")
ax[1].set_ylabel("angle initial maximal rattrapé (°)")
ax[1].set_title("LQR : R trop petit ⇒ saturation ⇒ bassin plus petit")
sauver(fig, "partie1", "p1_bonus_bassin_attraction")

# --- 4. marges (Nyquist de L(jω) = K (jωI - A)⁻¹ B) ------------------------------------------
A, B = matrices_lineaires()
w = np.geomspace(0.01, 1000, 6000)
K_pid = np.array([[-15, 225, -7.5, 22.5]])
fig, ax = plt.subplots(1, 2, figsize=(12, 5))
marges = {}
for i, (nom, K) in enumerate([("Double PID (sans filtres)", K_pid), ("LQR nominal", K0)]):
    L = np.array([(K @ np.linalg.solve(1j * wk * np.eye(4) - A, B))[0, 0] for wk in w])
    ax[0].plot(L.real, L.imag, color=C[i + 1], label=nom)
    ax[0].plot(L.real, -L.imag, color=C[i + 1], lw=0.8, ls=":")
    # marge de phase : pour chaque passage |L| = 1, angle entre L(jωc) et le point -1
    croisements = np.where(np.diff(np.sign(np.abs(L) - 1)) != 0)[0]
    cand = [(180 - abs(np.degrees(np.angle(L[k]))), w[k]) for k in croisements]
    pm, wc = min(cand)
    marges[nom] = (pm, wc, np.radians(pm) / wc)
    ax[1].semilogx(w, 20 * np.log10(np.abs(1 + L)), color=C[i + 1], label=nom)
th = np.linspace(0, 2 * np.pi, 200)
ax[0].plot(-1 + np.cos(th), np.sin(th), "--", color=GRIS, lw=1, label="cercle unité centré en −1")
ax[0].plot(-1, 0, "+", color=C[7], ms=14, mew=2)
ax[0].set_xlim(-4, 2)
ax[0].set_ylim(-3, 3)
ax[0].set_aspect("equal")
ax[0].set_title("Nyquist de la boucle ouverte (coupée à l'entrée moteur)")
ax[0].legend(fontsize=8, loc="lower left")
ax[1].axhline(0, color=GRIS, ls="--", lw=1)
ax[1].set_xlabel("ω (rad/s)")
ax[1].set_ylabel("|1 + L(jω)| (dB)")
ax[1].set_title("Inégalité de Kalman : |1+L| ≥ 1 (0 dB) pour le LQR")
ax[1].legend(fontsize=8)
sauver(fig, "partie1", "p1_bonus_marges_nyquist")

with open(os.path.join(OUT, "p1_bonus_robustesse.md"), "w") as f:
    f.write("| Contrôleur | " + " | ".join(s[0] for s in SCEN) + " |\n|---|" + "---|" * len(SCEN) + "\n")
    for i, (nc, _) in enumerate(CTRL):
        f.write(f"| {nc} | " + " | ".join(t.replace("\n", " ") for t in TXT[i]) + " |\n")
    f.write("\nRetard maximal toléré (ms) : " + ", ".join(f"{k} {v:.1f}" for k, v in lim.items()) + "\n")
    f.write("\nBassin d'attraction (°) : " + ", ".join(f"{k} {v:.1f}" for k, v in res_bassin.items()) + "\n")
    f.write("\nBassin LQR vs R : " + ", ".join(f"R={r:g}: {b:.1f}°" for r, b in zip(Rs, bassin_R)) + "\n")
    for k, (pm, wc, dm) in marges.items():
        f.write(f"\n{k} : marge de phase {pm:.0f}° à ωc = {wc:.1f} rad/s -> marge de retard ≈ {1000 * dm:.0f} ms\n")
print(open(os.path.join(OUT, "p1_bonus_robustesse.md")).read())
