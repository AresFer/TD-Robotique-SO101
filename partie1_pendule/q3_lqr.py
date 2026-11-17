# -*- coding: utf-8 -*-
"""
Q3 : remplacer le PID par un LQR (pyro), comparer PID / LQR, faire varier Q et R.

   J = ∫ ( xᵀ Q x + uᵀ R u ) dt ,   x = [θ, α, θ', α'],  u = θ''  ->  u = -K x
   K = R⁻¹ Bᵀ S  avec S solution de Riccati  AᵀS + SA - SBR⁻¹BᵀS + Q = 0  (pyro)

Figures (figures/partie1/) :
  p1_q3_lqr_pyro_boucle_fermee   simulation 100 % pyro (ctl + sys) du modèle NON linéaire
  p1_q3_comparaison_pid_lqr      Shawn / double PID / LQR / LQI sur le même scénario
  p1_q3_balayage_Q_R             influence de qθ, qα, qθ', R
  p1_q3_lieu_des_poles_R         pôles en boucle fermée et gains K quand R varie
  p1_q3_compromis_effort         compromis effort / rapidité (front de Pareto)
Lancer :  python partie1_pendule/q3_lqr.py
"""
import os
import sys

import numpy as np

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [ICI, os.path.dirname(ICI)]
from outils.style import plt, sauver, C, GRIS  # noqa: E402
from commandes import (PIDShawn, DoublePID, LQR, LQI, gain_lqr, gain_lqi,  # noqa: E402
                       simuler, metriques, DEG)
from modele_pendule import matrices_lineaires, PenduleRotatifEdukit  # noqa: E402

OUT = os.path.join(os.path.dirname(ICI), "figures", "partie1")
A, B = matrices_lineaires()
Q0 = np.diag([100.0, 100.0, 1.0, 1.0])     # LQR nominal : qθ, qα, qθ', qα'
R0 = 1.0
K0 = gain_lqr(Q0, R0)
md = []
print("K nominal (pyro) =", np.round(K0, 2))
print("pôles boucle fermée :", np.round(np.linalg.eigvals(A - B @ K0), 2))

# =========================================================================================
# 1. Simulation 100 % pyro : contrôleur LQR pyro + modèle non linéaire pyro
# =========================================================================================
from pyro.dynamic.statespace import StateSpaceSystem  # noqa: E402
from pyro.analysis.costfunction import QuadraticCostFunction  # noqa: E402
from pyro.control.lqr import synthesize_lqr_controller  # noqa: E402

ss = StateSpaceSystem(A, B, np.eye(4), np.zeros((4, 1)))
cf = QuadraticCostFunction(4, 1)
cf.Q, cf.R = Q0, np.array([[R0]])
ctl = synthesize_lqr_controller(ss, cf)
fig, ax = plt.subplots(1, 3, figsize=(15, 3.8))
for i, a0 in enumerate([2, 6, 10, 14]):
    nl = PenduleRotatifEdukit()
    cl = ctl + nl                                    # boucle fermée NON linéaire (pyro)
    cl.x0 = np.array([0, a0 * DEG, 0, 0])
    tr = cl.compute_trajectory(tf=4.0, n=2001)
    lin = ctl + ss                                   # même contrôleur sur le modèle linéaire
    lin.x0 = cl.x0
    trl = lin.compute_trajectory(tf=4.0, n=2001)
    u = np.array([float(-(K0 @ x)[0]) for x in tr.x])
    ax[0].plot(tr.t, tr.x[:, 1] / DEG, color=C[i], label=f"α0 = {a0}°")
    ax[0].plot(trl.t, trl.x[:, 1] / DEG, ":", color=C[i], lw=1.2)
    ax[1].plot(tr.t, tr.x[:, 0] / DEG, color=C[i])
    ax[1].plot(trl.t, trl.x[:, 0] / DEG, ":", color=C[i], lw=1.2)
    ax[2].plot(tr.t, u, color=C[i])
ax[0].set_title("α pendule (— non linéaire, ··· linéaire)")
ax[1].set_title("θ rotor")
ax[2].set_title("commande u = θ'' (non saturée)")
ax[0].set_ylabel("α (°)")
ax[1].set_ylabel("θ (°)")
ax[2].set_ylabel("u (rad/s²)")
for a in ax:
    a.set_xlabel("t (s)")
ax[0].legend()
fig.suptitle(f"Q3 — LQR pyro en boucle fermée sur le modèle pyro non linéaire   K = {np.round(K0[0], 1)}",
             fontweight="bold")
sauver(fig, "partie1", "p1_q3_lqr_pyro_boucle_fermee")

# =========================================================================================
# 2. Comparaison PID / LQR / LQI sur le même scénario réaliste
# =========================================================================================


def ref_rotor(t):
    return 45 * DEG if t >= 5.0 else 0.0


scen = dict(T=10, x0=(0, 3 * DEG, 0, 0), poussee=(2.5, 0.6), ref_theta=ref_rotor, calage=0.5 * DEG)
K_lqi = gain_lqi(Q0, R0, 100.0)
ctrls = [("PID Shawn (Δt 40 ms)", lambda: PIDShawn(Kp=0.7, Ki=0.005, Ts=0.04)),
         ("Double PID EDUKIT", lambda: DoublePID()),
         ("LQR pyro", lambda: LQR(K0)),
         ("LQI pyro (+ intégrale)", lambda: LQI(K_lqi))]
fig, ax = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
ax[1].plot([0, 5, 5, 10], [0, 0, 45, 45], "--", color=GRIS, lw=1.2, label="consigne rotor")
md.append("| Contrôleur | Tombe ? | max\\|α\\| après poussée (°) | RMS α (°) | θ final (°) | erreur statique θ (°) | max\\|u\\| (rad/s²) | ∫u² dt |")
md.append("|---|---|---|---|---|---|---|---|")
for i, (nom, fab) in enumerate(ctrls):
    lg = simuler(fab(), **scen)
    ax[0].plot(lg["t"], lg["alpha"] / DEG, color=C[i], label=nom + (f"  ({lg['raison']})" if lg["tombe"] else ""))
    ax[1].plot(lg["t"], lg["theta"] / DEG, color=C[i], label=nom)
    ax[2].plot(lg["t"], lg["u"], color=C[i], lw=0.9, label=nom)
    m = metriques(lg, t0=2.5)
    th_fin = lg["theta"][-1] / DEG
    md.append(f"| {nom} | {('oui : ' + lg['raison']) if m['tombe'] else 'non'} | {m['alpha_max']:.2f} | {m['rms_alpha']:.2f} | "
              f"{th_fin:.1f} | {th_fin - 45:+.1f} | {m['u_max']:.1f} | {m['energie']:.0f} |")
ax[0].set_ylabel("α (°)")
ax[1].set_ylabel("θ (°)")
ax[2].set_ylabel("u (rad/s²)")
ax[2].set_xlabel("t (s)")
ax[0].legend(ncol=2, loc="lower right")
ax[1].legend(ncol=3, loc="upper left", fontsize=7.5)
ax[1].set_ylim(-60, 120)
for a in ax:
    a.axvline(2.5, color=GRIS, ls=":", lw=1)
    a.axvline(5.0, color=GRIS, ls=":", lw=1)
ax[0].set_title("Q3 — Comparaison sur un même scénario : départ 3°, poussée à 2.5 s, consigne rotor 45° à 5 s, "
                "socle incliné 0.5°", fontsize=10)
sauver(fig, "partie1", "p1_q3_comparaison_pid_lqr")

# =========================================================================================
# 3. Balayage des coefficients de Q et R
# =========================================================================================
scen2 = dict(T=10, x0=(0, 3 * DEG, 0, 0), poussee=(2.5, 0.6), ref_theta=ref_rotor)
bal = [("qθ (poids position rotor)", 0, [1, 10, 100, 1000]),
       ("qα (poids angle pendule)", 1, [1, 100, 1e3, 1e4]),
       ("qθ' (poids vitesse rotor)", 2, [0.01, 1, 10, 100]),
       ("R (poids de la commande)", None, [0.01, 0.1, 1, 10])]
fig, ax = plt.subplots(3, 4, figsize=(17, 9), sharex=True)
md2 = ["| Variation | K = [kθ, kα, kθ', kα'] | pôle le plus lent | RMS α (°) | max\\|u\\| | ∫u² |", "|---|---|---|---|---|---|"]
for col, (nom, k, valeurs) in enumerate(bal):
    for i, v in enumerate(valeurs):
        Q, R = Q0.copy(), R0
        if k is None:
            R = v
        else:
            Q[k, k] = v
        K = gain_lqr(Q, R)
        lg = simuler(LQR(K), **scen2)
        lab = f"{nom.split()[0]} = {v:g}" + ("  (tombe)" if lg["tombe"] else "")
        ax[0, col].plot(lg["t"], lg["alpha"] / DEG, color=C[i], label=lab)
        ax[1, col].plot(lg["t"], lg["theta"] / DEG, color=C[i])
        ax[2, col].plot(lg["t"], lg["u"], color=C[i], lw=0.8)
        ev = np.linalg.eigvals(A - B @ K)
        m = metriques(lg, t0=2.5)
        md2.append(f"| {nom.split()[0]} = {v:g} | {np.round(K[0], 1).tolist()} | {-ev.real.max():.2f} | "
                   f"{m['rms_alpha']:.2f} | {m['u_max']:.1f} | {m['energie']:.0f} |")
    ax[0, col].set_title(nom)
    ax[0, col].legend(fontsize=7.5, loc="lower right")
    ax[1, col].plot([0, 5, 5, 10], [0, 0, 45, 45], "--", color=GRIS, lw=1)
    ax[2, col].set_xlabel("t (s)")
    ax[0, col].set_ylim(-8, 8)
    ax[1, col].set_ylim(-40, 90)
    ax[2, col].set_ylim(-62, 62)
ax[0, 0].set_ylabel("α (°)")
ax[1, 0].set_ylabel("θ (°)")
ax[2, 0].set_ylabel("u (rad/s²)")
fig.suptitle("Q3 — Influence des coefficients de Q et R (référence : Q = diag(100, 100, 1, 1), R = 1)",
             fontweight="bold")
sauver(fig, "partie1", "p1_q3_balayage_Q_R")

# =========================================================================================
# 4. Lieu des pôles et gains quand R varie (« LQR root locus »)
# =========================================================================================
Rs = np.geomspace(1e-4, 1e2, 120)
poles = np.array([np.sort_complex(np.linalg.eigvals(A - B @ gain_lqr(Q0, r, avec_pyro=False))) for r in Rs])
Ks = np.array([gain_lqr(Q0, r, avec_pyro=False)[0] for r in Rs])
fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
for j in range(4):
    sc = ax[0].scatter(poles[:, j].real, poles[:, j].imag, c=np.log10(Rs), cmap="viridis", s=10)
ol = np.linalg.eigvals(A)
ax[0].scatter(ol.real, ol.imag, marker="x", color=C[7], s=70, lw=2, label="pôles boucle ouverte", zorder=3)
ax[0].axvline(0, color=GRIS, lw=1)
ax[0].set_xlabel("Re")
ax[0].set_ylabel("Im")
ax[0].set_title("Pôles de A − BK quand R varie de 1e-4 à 1e2")
ax[0].set_xlim(-45, 10)
ax[0].legend(loc="upper left")
fig.colorbar(sc, ax=ax[0], label="log10(R)")
noms = ["kθ", "kα", "kθ'", "kα'"]
for j in range(4):
    ax[1].loglog(Rs, np.abs(Ks[:, j]), color=C[j], label=f"|{noms[j]}|")
ax[1].axvline(R0, color=GRIS, ls=":", lw=1)
ax[1].set_xlabel("R")
ax[1].set_ylabel("|gain|")
ax[1].set_title("Gains LQR en fonction de R (Q fixé)")
ax[1].legend()
sauver(fig, "partie1", "p1_q3_lieu_des_poles_R")

# =========================================================================================
# 5. Compromis effort / performance (simulation non linéaire, saturation comprise)
# =========================================================================================
def t_montee(lg, t0=5.0, cible=45.0):
    i = np.searchsorted(lg["t"], t0)
    th = lg["theta"][i:] / DEG
    k = np.argmax(th >= 0.9 * cible)
    return lg["t"][i + k] - t0 if th.max() >= 0.9 * cible else np.nan


fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
for i, qt in enumerate([10, 100, 1000]):
    E, TM, RA = [], [], []
    for r in np.geomspace(0.003, 30, 14):
        Q = Q0.copy()
        Q[0, 0] = qt
        lg = simuler(LQR(gain_lqr(Q, r, avec_pyro=False)), **scen2)
        if lg["tombe"]:
            continue
        m = metriques(lg, t0=5.0)
        E.append(m["energie"])
        TM.append(t_montee(lg))
        RA.append(m["alpha_max"])
    ax[0].plot(E, TM, "o-", color=C[i], ms=5, label=f"qθ = {qt}  (R de 30 → 0.003)")
    ax[1].plot(E, RA, "o-", color=C[i], ms=5, label=f"qθ = {qt}")
lg = simuler(DoublePID(), **scen2)
m = metriques(lg, t0=5.0)
ax[0].plot(m["energie"], t_montee(lg), "*", color=C[7], ms=15, ls="none", label="Double PID EDUKIT")
ax[1].plot(m["energie"], m["alpha_max"], "*", color=C[7], ms=15, ls="none", label="Double PID EDUKIT")
for a in ax:
    a.set_xscale("log")
    a.set_xlabel("énergie de commande ∫u² dt pendant le déplacement du rotor")
ax[0].set_ylabel("temps de montée du rotor à 90 % (s)")
ax[1].set_ylabel("inclinaison max du pendule pendant le déplacement (°)")
ax[0].set_title("Rapidité du rotor contre effort")
ax[1].set_title("Prix à payer : le pendule s'incline davantage")
ax[0].legend(fontsize=8)
fig.suptitle("Q3 — Compromis du LQR : un rotor plus nerveux coûte de l'énergie ET incline le pendule",
             fontweight="bold")
sauver(fig, "partie1", "p1_q3_compromis_effort")

with open(os.path.join(OUT, "p1_q3_metriques.md"), "w") as f:
    f.write(f"K nominal (pyro, Q = diag(100,100,1,1), R = 1) = {np.round(K0[0], 2).tolist()}\n\n")
    f.write(f"Pôles en boucle fermée : {np.round(np.linalg.eigvals(A - B @ K0), 2).tolist()}\n\n")
    f.write(f"K LQI (q_int = 100) = {np.round(K_lqi[0], 2).tolist()}\n\n")
    f.write("\n".join(md) + "\n\n" + "\n".join(md2) + "\n")
print("\n".join(md))
print("\n".join(md2))
