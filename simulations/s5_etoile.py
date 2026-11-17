# -*- coding: utf-8 -*-
"""
Q6 : LQR de A vers B puis parcours en étoile — exécute partie2_so101/lqr_etoile.py (original,
non modifié) sur le jumeau numérique, pour les deux lectures de « parcours en étoile ».

Figures :
  p2_q6_etoile_vue_dessus      pentagramme et rayons : consigne, état LQR, mesure du jumeau
  p2_q6_etoile_reglages        influence de Q (poids position) et de VEL_MAX sur la précision
  gif/q6_pentagramme.gif, gif/q6_rayons.gif, p2_q6_etoile_rendu_*.png (rendu MuJoCo)
Lancer :  python simulations/s5_etoile.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from outils.banc import executer, RACINE, EX  # noqa: E402
from outils.style import plt, sauver, C, GRIS  # noqa: E402

FIG = os.path.join(RACINE, "figures", "partie2")
GIF = os.path.join(RACINE, "figures", "gif")
Q_HOME = np.array([0.0, -1.0, 1.0, 1.2, 1.52, 0.0])
sys.path.insert(0, EX)


def dist_polyligne(P, S):
    """Distance de chaque point de P à la polyligne S (dans le plan xy)."""
    d = np.full(len(P), np.inf)
    for a, b in zip(S[:-1], S[1:]):
        ab = b[:2] - a[:2]
        t = np.clip(((P[:, :2] - a[:2]) @ ab) / (ab @ ab), 0, 1)
        proj = a[:2] + t[:, None] * ab
        d = np.minimum(d, np.linalg.norm(P[:, :2] - proj, axis=1))
    return d


def lancer(pattern, reglages=None, gif=None, actif=False):
    r = dict(PATTERN=pattern)
    r.update(reglages or {})
    mod, logs = executer("lqr_etoile", "real", "", None, gif=gif, reglages=r, q0_jumeau=Q_HOME,
                         kw_vue=dict(actif=actif, tous_les=3, largeur=420, hauteur=315,
                                     camera=dict(azimuth=180, elevation=-62, distance=0.55,
                                                 lookat=(0.22, 0.0, 0.02))))
    L = logs["retour"]
    from astar_planner import ArmPlanner
    P = ArmPlanner(os.path.join(EX, "scene_so101.xml"), [], verbose=False)
    q, meas, seg = np.array(L["q"]), np.array(L["meas"]), np.array(L["seg"])
    tip = np.array([P.fk(x)[0] for x in q])
    tipm = np.array([P.fk(x)[0] for x in meas])
    S = np.array(mod.sommets_etoile())
    m = seg > 1        # après « A -> B »
    return dict(mod=mod, L=L, tip=tip, tipm=tipm, S=S, m=m, vue=logs["vue"],
                err=dist_polyligne(tip[m], S), errm=dist_polyligne(tipm[m], S),
                dz=tipm[m, 2] - S[0, 2], T=L["t"][-1])


if __name__ == "__main__":
    res = {}
    for pat in ("pentagramme", "rayons"):
        print(f"=== {pat} ===")
        res[pat] = lancer(pat, gif=os.path.join(GIF, f"q6_{pat}.gif"), actif=True)
        fr = res[pat]["vue"].derniere.frames
        if fr:
            import PIL.Image
            PIL.Image.fromarray(fr[-1]).save(os.path.join(FIG, f"p2_q6_etoile_rendu_{pat}.png"))

    fig, ax = plt.subplots(1, 2, figsize=(13, 5.6))
    for a, pat in zip(ax, res):
        r = res[pat]
        a.plot(r["S"][:, 1], r["S"][:, 0], "--", color=GRIS, lw=1.4, label="étoile idéale")
        a.plot(r["tip"][r["m"], 1], r["tip"][r["m"], 0], color=C[0], lw=1.6, label="pince : état LQR")
        a.plot(r["tipm"][r["m"], 1], r["tipm"][r["m"], 0], color=C[1], lw=1.4, label="pince : mesure du jumeau")
        a.plot(r["tip"][~r["m"], 1], r["tip"][~r["m"], 0], ":", color=C[2], lw=1.2, label="départ -> A -> B")
        a.invert_xaxis()
        a.set_aspect("equal")
        a.set_xlabel("y (m)")
        a.set_ylabel("x (m)")
        a.set_title(f"« {pat} » — écart moyen à l'étoile : état {1000 * r['err'].mean():.1f} mm, "
                    f"mesure {1000 * r['errm'].mean():.1f} mm", fontsize=10)
        a.legend(fontsize=8, loc="lower left")
    fig.suptitle("Q6 — Parcours en étoile par LQR (vue de dessus, pince verticale à 4 cm de la table)",
                 fontweight="bold")
    sauver(fig, "partie2", "p2_q6_etoile_vue_dessus")

    # --- réglages : Q_POS et VEL_MAX -------------------------------------------------------
    print("=== réglages ===")
    lignes = ["| Réglage | durée (s) | écart moyen état (mm) | écart max état (mm) | écart moyen mesure (mm) | "
              "écart max mesure (mm) | affaissement z moyen (mm) |", "|---|---|---|---|---|---|---|"]
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
    for k, (nom, cle, valeurs) in enumerate([("Q_POS (poids de la position)", "Q_POS", [1, 9, 81, 729]),
                                             ("VEL_MAX (rad/s)", "VEL_MAX", [0.5, 1.5, 3.0, 6.0])]):
        xs, e_state, e_meas, durees = [], [], [], []
        for v in valeurs:
            reg = {cle: v}
            if cle == "VEL_MAX":
                reg["ACC_MAX"] = 2.7 * v
            r = lancer("pentagramme", reg)
            xs.append(v)
            e_state.append(1000 * r["err"].mean())
            e_meas.append(1000 * r["errm"].mean())
            durees.append(r["T"])
            lignes.append(f"| {cle} = {v:g} | {r['T']:.1f} | {1000 * r['err'].mean():.1f} | {1000 * r['err'].max():.1f} | "
                          f"{1000 * r['errm'].mean():.1f} | {1000 * r['errm'].max():.1f} | {1000 * r['dz'].mean():+.1f} |")
        ax[k].plot(xs, e_state, "o-", color=C[0], label="état LQR")
        ax[k].plot(xs, e_meas, "o-", color=C[1], label="mesure (jumeau)")
        for x, e, d in zip(xs, e_meas, durees):
            ax[k].annotate(f"{d:.0f} s", (x, e), textcoords="offset points", xytext=(4, 6), fontsize=8, color=GRIS)
        ax[k].set_xscale("log")
        ax[k].set_xlabel(nom)
        ax[k].set_ylabel("écart moyen à l'étoile (mm)")
        ax[k].legend()
    ax[0].set_title("Poids Q de la position (R = 1)")
    ax[1].set_title("Vitesse max de la loi horaire (ACC_MAX = 2.7·VEL_MAX)\n(étiquettes = durée du parcours)")
    sauver(fig, "partie2", "p2_q6_etoile_reglages")
    with open(os.path.join(FIG, "p2_q6_etoile_metriques.md"), "w") as f:
        for pat, r in res.items():
            f.write(f"- {pat} : durée {r['T']:.1f} s, écart moyen état {1000 * r['err'].mean():.1f} mm, "
                    f"mesure {1000 * r['errm'].mean():.1f} mm (max {1000 * r['errm'].max():.1f} mm), "
                    f"affaissement moyen {1000 * r['dz'].mean():+.1f} mm\n")
        f.write("\n" + "\n".join(lignes) + "\n")
    print("\n".join(lignes))
