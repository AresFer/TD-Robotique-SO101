# -*- coding: utf-8 -*-
"""
Q3 (« vérifiez en simulation la dynamique du système ») : validation du modèle EDUKIT
AVANT de concevoir le moindre correcteur.

1. Oscillation libre pendule en bas  -> la période doit valoir 2π/ωg (ωg² = 55.85).
2. Chute libre depuis la verticale   -> croissance exponentielle en exp(+ωg t) au début.
3. Linéarisation numérique pyro vs matrices analytiques + pôles / zéros.

Lancer :  python partie1_pendule/validation_modele.py   (figures -> figures/partie1/)
"""
import os
import sys

import numpy as np

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [ICI, os.path.dirname(ICI)]
from outils.style import plt, sauver, C, GRIS  # noqa: E402
from modele_pendule import (PARAM_EDUKIT, matrices_lineaires, rk4,  # noqa: E402
                            modele_pyro_lineaire, PYRO)

DEG = np.pi / 180
p = PARAM_EDUKIT
wg = np.sqrt(p["wg2"])


def simuler_libre(x0, T, dt=0.0005):
    x = np.array(x0, float)
    out = [x.copy()]
    for _ in range(int(T / dt)):
        x = rk4(x, 0.0, dt)
        out.append(x.copy())
    return np.arange(len(out)) * dt, np.array(out)


fig, ax = plt.subplots(1, 3, figsize=(14, 3.8))

# --- 1. oscillation libre autour de la position basse (α = π) ----------------------------
t, X = simuler_libre([0, np.pi - 10 * DEG, 0, 0], 6.0)
dev = (X[:, 1] - np.pi) / DEG
ax[0].plot(t, dev, color=C[0], label="modèle non linéaire")
env = -10 * np.exp(-p["gamma"] / 2 * t)
ax[0].plot(t, env, "--", color=GRIS, lw=1.2, label="enveloppe e^(−γt/2)")
ax[0].plot(t, -env, "--", color=GRIS, lw=1.2)
passages = t[1:][(np.sign(dev[1:]) != np.sign(dev[:-1]))]
T_mes = 2 * np.mean(np.diff(passages))
T_th = 2 * np.pi / np.sqrt(p["wg2"] - p["gamma"] ** 2 / 4)
ax[0].set_title("Pendule en bas : oscillation libre")
ax[0].set_xlabel("t (s)")
ax[0].set_ylabel("écart à la verticale basse (°)")
ax[0].text(0.98, 0.04, f"période simulée {T_mes:.3f} s\nthéorie 2π/ωd = {T_th:.3f} s",
           transform=ax[0].transAxes, ha="right", fontsize=8.5,
           bbox=dict(fc="white", ec="#ddd"))
ax[0].legend(loc="upper right")

# --- 2. chute depuis la verticale haute ---------------------------------------------------
t, X = simuler_libre([0, 0.5 * DEG, 0, 0], 0.9)
A, B = matrices_lineaires()
lam = np.max(np.linalg.eigvals(A).real)
# solution exacte du linéaire : α(0)=α0, α'(0)=0  ->  combinaison des deux modes réels
l1, l2 = np.sort(np.linalg.eigvals(A[[1, 3]][:, [1, 3]]).real)
c1 = -l2 / (l1 - l2) * 0.5
c2 = l1 / (l1 - l2) * 0.5
al_lin = c1 * np.exp(l1 * t) + c2 * np.exp(l2 * t)
ax[1].semilogy(t, np.abs(X[:, 1]) / DEG, color=C[0], label="non linéaire")
ax[1].semilogy(t, np.abs(al_lin), "--", color=C[1], label="linéarisé")
ax[1].axhline(20, color=GRIS, lw=1, ls=":")
ax[1].text(0.02, 22, "garde ±20°", color=GRIS, fontsize=8)
t_double = np.log(2) / lam
ax[1].set_title("Pendule en haut : chute libre depuis 0.5°")
ax[1].set_xlabel("t (s)")
ax[1].set_ylabel("|α| (°, échelle log)")
ax[1].text(0.98, 0.04, f"pôle instable λ = +{lam:.2f} rad/s\ntemps de doublement {1000 * t_double:.0f} ms",
           transform=ax[1].transAxes, ha="right", fontsize=8.5, bbox=dict(fc="white", ec="#ddd"))
ax[1].legend(loc="upper left")
t_chute = t[np.argmax(np.abs(X[:, 1]) > 20 * DEG)]

# --- 3. pôles en boucle ouverte -------------------------------------------------------------
ev = np.linalg.eigvals(A)
ax[2].axvline(0, color=GRIS, lw=1)
ax[2].axhline(0, color=GRIS, lw=1)
ax[2].scatter(ev.real, ev.imag, s=70, marker="x", color=C[7], lw=2, label="pôles de A", zorder=3)
for e in np.unique(np.round(ev.real, 3)):
    mult = int(np.sum(np.isclose(ev.real, e, atol=1e-3)))
    ax[2].annotate(f"{e:+.2f}" + (f" (×{mult})" if mult > 1 else ""), (e, 0),
                   textcoords="offset points", xytext=(0, 9), ha="center", fontsize=8.5)
ax[2].set_xlim(-10, 10)
ax[2].set_ylim(-3, 3)
ax[2].set_title("Pôles du modèle linéarisé (boucle ouverte)")
ax[2].set_xlabel("Re (rad/s)")
ax[2].set_ylabel("Im (rad/s)")
ax[2].text(0.5, 0.88, "double pôle en 0 : rotor = double intégrateur", transform=ax[2].transAxes,
           ha="center", fontsize=8.5)
ax[2].legend(loc="lower right")
sauver(fig, "partie1", "p1_validation_modele")

# --- 4. linéarisation numérique pyro -------------------------------------------------------
lignes = ["| Grandeur | Valeur |", "|---|---|",
          f"| Période d'oscillation simulée (pendule en bas) | {T_mes:.3f} s |",
          f"| Période théorique 2π/√(ωg² − γ²/4) | {T_th:.3f} s |",
          f"| Pôle instable (pendule en haut) | +{lam:.3f} rad/s |",
          f"| Temps de doublement de l'angle | {1000 * t_double:.0f} ms |",
          f"| Temps de chute 0.5° → 20° (sans commande) | {t_chute:.2f} s |"]
if PYRO:
    ss = modele_pyro_lineaire()
    lignes.append(f"| max\\|A_pyro − A_analytique\\| | {np.abs(ss.A - A).max():.1e} |")
    lignes.append(f"| max\\|B_pyro − B_analytique\\| | {np.abs(ss.B - B).max():.1e} |")
with open(os.path.join(os.path.dirname(ICI), "figures", "partie1", "p1_validation_modele.md"), "w") as f:
    f.write("\n".join(lignes) + "\n")
print("\n".join(lignes))
