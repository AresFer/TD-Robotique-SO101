# -*- coding: utf-8 -*-
"""
Q1 : faire tourner le PID (code Shawn Hymel) — ici sur le modèle EDUKIT non linéaire.
Q2 : tester différentes valeurs de PID et les afficher dans des graphes.

Deux architectures sont étudiées :
  A. PID de Shawn Hymel : PID sur l'angle du pendule SEUL, sortie = déplacement du moteur.
  B. Double PID du firmware EDUKIT : PID pendule + PID rotor, sortie = accélération du rotor.

Lancer :  python partie1_pendule/q1_q2_pid.py      (≈ 1 min ; figures -> figures/partie1/)
"""
import os
import sys

import numpy as np

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [ICI, os.path.dirname(ICI)]
from outils.style import plt, sauver, C, GRIS  # noqa: E402
from commandes import PIDShawn, DoublePID, simuler, metriques, DEG  # noqa: E402
from modele_pendule import matrices_lineaires  # noqa: E402

TS_SHAWN = 0.040          # période de boucle supposée (USB-série + Arduino) — voir figure Δt
OUT = os.path.join(os.path.dirname(ICI), "figures", "partie1")
lignes_md = []


def trace_alpha_theta(axa, axt, log, couleur, label, ls="-"):
    axa.plot(log["t"], log["alpha"] / DEG, ls, color=couleur, label=label, lw=1.6)
    axt.plot(log["t"], log["theta"] / DEG, ls, color=couleur, label=label, lw=1.6)
    if log["tombe"]:
        axa.plot(log["t"][-1], log["alpha"][-1] / DEG, "x", color=couleur, ms=9, mew=2)
        axt.plot(log["t"][-1], log["theta"][-1] / DEG, "x", color=couleur, ms=9, mew=2)


# =========================================================================================
# A1. Q1 — PID de Shawn Hymel, gains du notebook (Kp=1, Ki=0.005, Kd=0)
# =========================================================================================
print("Q1 : PID Shawn Hymel nominal")
scen = dict(T=12, x0=(0, 1 * DEG, 0, 0), poussee=(4.0, 0.3))
log0 = simuler(PIDShawn(Ts=TS_SHAWN), **scen)
log1 = simuler(PIDShawn(Ts=TS_SHAWN), calage=0.5 * DEG, **scen)
fig, ax = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
for lg, c, lab in ((log0, C[0], "verticale parfaitement calée"),
                   (log1, C[1], "socle incliné de 0.5° (cas réel)")):
    ax[0].plot(lg["t"], lg["alpha"] / DEG, color=c, label=lab)
    ax[1].plot(lg["t"], lg["theta"] / DEG, color=c, label=lab)
    ax[2].plot(lg["t"], lg["u"], color=c, lw=1.0, label=lab)
ax[0].axvline(4.0, color=GRIS, ls=":", lw=1)
ax[0].text(4.05, ax[0].get_ylim()[1] * 0.8, "poussée", color=GRIS, fontsize=8)
ax[0].set_ylabel("α pendule (°)")
ax[1].set_ylabel("θ rotor (°)")
ax[2].set_ylabel("accélération rotor (rad/s²)")
ax[2].set_xlabel("t (s)")
ax[0].legend(loc="lower right")
ax[0].set_title(f"Q1 — PID de Shawn Hymel (Kp=1, Ki=0.005, Kd=0, Δt={1000 * TS_SHAWN:.0f} ms) : "
                "le pendule tient, mais le rotor n'est pas asservi")
sauver(fig, "partie1", "p1_q1_pid_shawn_nominal")
for nom, lg in (("calé", log0), ("incliné 0.5°", log1)):
    m = metriques(lg)
    lignes_md.append(f"| Shawn nominal ({nom}) | {'oui' if m['tombe'] else 'non'} | {m['alpha_max']:.2f} | "
                     f"{m['theta_max']:.1f} | {m['rms_alpha']:.2f} |")
derive = (log1["theta"][-1] - log1["theta"][int(len(log1['t']) / 2)]) / DEG / (log1["t"][-1] - log1["t"][int(len(log1['t']) / 2)])
print(f"  dérive du rotor (socle incliné) : {derive:.1f} °/s")

# =========================================================================================
# A2. Q2 — balayages Kp, Ki, Kd, Δt (Shawn)
# =========================================================================================
print("Q2 : balayages PID Shawn")
scen = dict(T=6, x0=(0, 1 * DEG, 0, 0), poussee=(3.0, 0.3))
fig, ax = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
balayages = [("Kp", [0.25, 0.5, 1.0, 1.5, 3.0], dict(Ki=0.005, Kd=0.0)),
             ("Ki", [0.0, 0.002, 0.005, 0.008, 0.02], dict(Kp=1.0, Kd=0.0)),
             ("Kd", [0.0, 5.0, 15.0, 40.0, 100.0], dict(Kp=1.0, Ki=0.005)),
             ("Δt", [0.010, 0.020, 0.040, 0.050, 0.070], dict(Kp=1.0, Ki=0.005, Kd=0.0))]
for a, (nom, valeurs, fixes) in zip(ax.flat, balayages):
    for i, v in enumerate(valeurs):
        if nom == "Δt":
            ctrl, lab = PIDShawn(Ts=v, **fixes), f"Δt = {1000 * v:.0f} ms"
        else:
            g = dict(fixes)
            g[nom] = v
            ctrl, lab = PIDShawn(Ts=TS_SHAWN, **g), f"{nom} = {v:g}"
        lg = simuler(ctrl, **scen)
        a.plot(lg["t"], lg["alpha"] / DEG, color=C[i], lw=1.5, label=lab + ("  (tombe)" if lg["tombe"] else ""))
        if lg["tombe"]:
            a.plot(lg["t"][-1], lg["alpha"][-1] / DEG, "x", color=C[i], ms=8, mew=2)
        m = metriques(lg)
        lignes_md.append(f"| Shawn {lab} | {'oui' if m['tombe'] else 'non'} | {m['alpha_max']:.2f} | "
                         f"{m['theta_max']:.1f} | {m['rms_alpha']:.2f} |")
    a.set_ylim(-21, 21)
    a.set_title(f"Effet de {nom}")
    a.set_ylabel("α (°)")
    a.legend(loc="lower right", fontsize=7.5)
for a in ax[1]:
    a.set_xlabel("t (s)")
fig.suptitle("Q2 — PID de Shawn Hymel : influence de chaque paramètre (départ 1°, poussée à t = 3 s)",
             fontweight="bold")
sauver(fig, "partie1", "p1_q2_pid_shawn_balayages")

# --- carte de stabilité Kp x Ki (simulation non linéaire complète) ------------------------
print("  carte de stabilité Kp x Ki ...")
kps = np.geomspace(0.15, 3, 24)
kis = np.geomspace(0.001, 0.025, 24)
carte = np.full((len(kis), len(kps)), np.nan)
for j, kp in enumerate(kps):
    for i, ki in enumerate(kis):
        lg = simuler(PIDShawn(Kp=kp, Ki=ki, Ts=TS_SHAWN), T=6, x0=(0, 1 * DEG, 0, 0), poussee=(3.0, 0.3))
        m = metriques(lg)
        carte[i, j] = np.nan if m["tombe"] else m["rms_alpha"]
fig, a = plt.subplots(figsize=(7, 5.2))
cm = plt.get_cmap("Blues_r").copy()
cm.set_bad("#e9e8e3")
im = a.pcolormesh(kps, kis, np.ma.masked_invalid(carte), cmap=cm, shading="nearest", vmin=0, vmax=3)
a.set_xscale("log")
a.set_yscale("log")
a.plot(1.0, 0.005, "*", color=C[1], ms=16, mec="white", label="gains du notebook (1, 0.005)")
a.set_xlabel("Kp (°/°)")
a.set_ylabel("Ki (°/(°·ms))")
a.set_title(f"Carte de stabilité du PID de Shawn (Δt = {1000 * TS_SHAWN:.0f} ms)\n"
            "gris = le pendule tombe ; couleur = RMS de α (°)")
fig.colorbar(im, ax=a, label="RMS α (°)")
a.legend(loc="upper right")
a.grid(False)
sauver(fig, "partie1", "p1_q2_pid_shawn_carte_stabilite")

# =========================================================================================
# B1. Double PID du firmware EDUKIT
# =========================================================================================
print("Q2 : double PID EDUKIT")


def ref_rotor(t):
    return 45 * DEG if t >= 5.0 else 0.0


scen = dict(T=10, x0=(0, 3 * DEG, 0, 0), poussee=(2.5, 0.6), ref_theta=ref_rotor, calage=0.5 * DEG)
lg = simuler(DoublePID(), **scen)
fig, ax = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
ax[0].plot(lg["t"], lg["alpha"] / DEG, color=C[0])
ax[0].set_ylabel("α pendule (°)")
ax[1].plot(lg["t"], lg["ref"] / DEG, "--", color=GRIS, label="consigne rotor")
ax[1].plot(lg["t"], lg["theta"] / DEG, color=C[1], label="θ rotor")
ax[1].set_ylabel("θ rotor (°)")
ax[1].legend(loc="upper left")
ax[2].plot(lg["t"], lg["u"], color=C[2], lw=1.0)
ax[2].set_ylabel("u = θ'' (rad/s²)")
ax[2].set_xlabel("t (s)")
for a in ax:
    a.axvline(2.5, color=GRIS, ls=":", lw=1)
    a.axvline(5.0, color=GRIS, ls=":", lw=1)
ax[0].text(2.55, 2.5, "poussée", color=GRIS, fontsize=8)
ax[0].text(5.05, 2.5, "consigne rotor 45°", color=GRIS, fontsize=8)
ax[0].set_title("Double PID EDUKIT (pendule Kp=225, Kd=22.5 ; rotor Kp=15, Kd=7.5 ; Ts = 2 ms)")
sauver(fig, "partie1", "p1_q2_double_pid_nominal")
idx = np.searchsorted(lg["t"], 5.0)
sous_ref = np.min(lg["theta"][idx:idx + 500]) / DEG
print(f"  réponse inverse du rotor après l'échelon : {sous_ref:.1f}° (avant de monter vers 45°)")
m = metriques(lg)
lignes_md.append(f"| Double PID nominal | {'oui' if m['tombe'] else 'non'} | {m['alpha_max']:.2f} | "
                 f"{m['theta_max']:.1f} | {m['rms_alpha']:.2f} |")

# --- balayages des 4 gains ------------------------------------------------------------------
fig, ax = plt.subplots(2, 4, figsize=(16, 6.5), sharex=True)
bal = [("Kp_p", [120, 160, 225, 400, 700]), ("Kd_p", [8, 15, 22.5, 40, 70]),
       ("Kp_r", [0, 5, 15, 30, 60]), ("Kd_r", [0, 3, 7.5, 15, 30])]
for col, (nom, valeurs) in enumerate(bal):
    for i, v in enumerate(valeurs):
        lg = simuler(DoublePID(**{nom: v}), **scen)
        lab = f"{nom} = {v:g}" + ("  (tombe)" if lg["tombe"] else "")
        trace_alpha_theta(ax[0, col], ax[1, col], lg, C[i], lab)
        m = metriques(lg)
        lignes_md.append(f"| Double PID {nom}={v:g} | {'oui' if m['tombe'] else 'non'} | {m['alpha_max']:.2f} | "
                         f"{m['theta_max']:.1f} | {m['rms_alpha']:.2f} |")
    ax[0, col].set_title(f"Effet de {nom}")
    ax[0, col].set_ylim(-21, 21)
    ax[1, col].plot(scen["T"] * np.array([0.5, 1]), [45, 45], "--", color=GRIS, lw=1)
    ax[1, col].set_ylim(-120, 160)
    ax[0, col].legend(fontsize=7, loc="lower right")
    ax[1, col].set_xlabel("t (s)")
ax[0, 0].set_ylabel("α pendule (°)")
ax[1, 0].set_ylabel("θ rotor (°)")
fig.suptitle("Q2 — Double PID EDUKIT : effet de chaque gain (poussée à 2.5 s, consigne rotor 45° à 5 s)",
             fontweight="bold")
sauver(fig, "partie1", "p1_q2_double_pid_balayages")

# --- carte de stabilité linéaire Kp_p x Kd_p (valeurs propres de A - BK) ------------------
A, B = matrices_lineaires()
kpp = np.linspace(0, 800, 161)
kdp = np.linspace(0, 100, 101)
marge = np.zeros((len(kdp), len(kpp)))
for i, kd in enumerate(kdp):
    for j, kp in enumerate(kpp):
        K = np.array([[-15, kp, -7.5, kd]])
        marge[i, j] = -np.max(np.linalg.eigvals(A - B @ K).real)
fig, a = plt.subplots(figsize=(7, 5.2))
im = a.pcolormesh(kpp, kdp, np.ma.masked_less_equal(marge, 0), cmap="Blues", shading="nearest", vmin=0, vmax=3)
a.contour(kpp, kdp, marge, levels=[0], colors=[C[7]], linewidths=1.8)
a.plot(225, 22.5, "*", color=C[1], ms=16, mec="white", label="gains firmware (225 ; 22.5)")
a.axvline(55.85 / 0.803, color=GRIS, ls="--", lw=1)
a.text(55.85 / 0.803 + 8, 92, "Kp_p > ωg²/ωr² ≈ 70", color=GRIS, fontsize=8.5)
a.set_xlabel("Kp pendule ((rad/s²)/rad)")
a.set_ylabel("Kd pendule ((rad/s²)/(rad/s))")
a.set_title("Double PID : zone de stabilité (modèle linéaire, rotor Kp=15, Kd=7.5)\n"
            "couleur = marge de stabilité −max Re(pôles) ; blanc = instable")
fig.colorbar(im, ax=a, label="−max Re(λ) (1/s)")
a.legend(loc="upper right")
a.grid(False)
sauver(fig, "partie1", "p1_q2_double_pid_zone_stabilite")

with open(os.path.join(OUT, "p1_q1_q2_metriques.md"), "w") as f:
    f.write("| Essai | Tombe ? | max\\|α\\| (°) | max\\|θ\\| (°) | RMS α (°) |\n|---|---|---|---|---|\n")
    f.write("\n".join(lignes_md) + "\n")
print("OK")
