# -*- coding: utf-8 -*-
"""Style commun des figures du compte rendu (palette catégorielle validée, traits fins,
grille discrète). Toutes les figures sont enregistrées dans figures/<partie>/."""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Palette catégorielle (ordre fixe, jamais recyclé) - validée daltonisme (ΔE CVD >= 8)
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
GRIS = "#8a8984"
ENCRE = "#0b0b0b"
ENCRE2 = "#52514e"
FOND = "#fcfcfb"

plt.rcParams.update({
    "figure.facecolor": FOND,
    "axes.facecolor": FOND,
    "savefig.facecolor": FOND,
    "axes.edgecolor": "#c9c8c2",
    "axes.labelcolor": ENCRE2,
    "axes.titlecolor": ENCRE,
    "axes.titleweight": "bold",
    "axes.titlesize": 11,
    "axes.labelsize": 9.5,
    "axes.grid": True,
    "grid.color": "#e6e5e0",
    "grid.linewidth": 0.8,
    "xtick.color": ENCRE2,
    "ytick.color": ENCRE2,
    "xtick.labelsize": 8.5,
    "ytick.labelsize": 8.5,
    "legend.fontsize": 8.5,
    "legend.frameon": False,
    "lines.linewidth": 1.8,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.prop_cycle": matplotlib.cycler(color=C),
    "font.family": "DejaVu Sans",
    "figure.dpi": 110,
})


def sauver(fig, partie, nom, dpi=130):
    """Enregistre la figure dans figures/<partie>/<nom>.png et la ferme."""
    dossier = os.path.join(RACINE, "figures", partie)
    os.makedirs(dossier, exist_ok=True)
    chemin = os.path.join(dossier, nom + ".png")
    fig.savefig(chemin, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print("  figure ->", os.path.relpath(chemin, RACINE))
    return chemin
