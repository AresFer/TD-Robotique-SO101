# -*- coding: utf-8 -*-
"""
Aller plus loin (Q8) : RRT et RRT* contre A* sur le transfert « au-dessus de A -> au-dessus de B »
du script pick_place_astar.py (même obstacle, même modèle de collision MuJoCo).

Indicateurs sur 10 graines aléatoires : succès, temps, nombre de nœuds, longueur articulaire,
longueur parcourue par la pince, plus grand saut articulaire. Figures dans figures/bonus/.
Lancer :  python simulations/s7_rrt.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from outils.banc import charger, effacer_points, RACINE  # noqa: E402
from outils.style import plt, sauver, C, GRIS  # noqa: E402

effacer_points()
mod = charger("pick_place_astar")
import astar_planner  # noqa: E402
from rrt_planner import RRT, densifier  # noqa: E402

P = astar_planner.ArmPlanner(mod.XML_PATH, mod.OBSTACLES, table_z=0.02, res=mod.GRID_RES, margin=mod.MARGIN,
                             clearance=mod.CLEARANCE, verbose=False)
qA, qB = mod.load_points(P)
qa, qb = mod.above(P, qA, mod.APPROACH_DZ), mod.above(P, qB, mod.APPROACH_DZ)
qa[5] = qb[5] = 0.0


def mesures(path):
    path = densifier(np.asarray(path))
    tip = np.array([P.fk(q)[0] for q in path])
    return dict(L_art=float(np.sum(np.linalg.norm(np.diff(path[:, :4], axis=0), axis=1))),
                L_pince=float(np.sum(np.linalg.norm(np.diff(tip, axis=0), axis=1))),
                z_max=float(tip[:, 2].max()), tip=tip)


print("A* ...")
path_a, _ = P.plan(qa, qb)
sA = P.stats[-1]
mA = mesures(path_a)
lignes = ["| Méthode | succès | temps moyen | nœuds / cases | longueur articulaire (rad) | longueur pince (cm) |",
          "|---|---|---|---|---|---|",
          f"| A* (grille 1 cm, lissé) | 1/1 | {sA['t_total']:.1f} s | {sA['expanded']} cases | {mA['L_art']:.2f} | "
          f"{100 * mA['L_pince']:.1f} |"]
resultats = {"A*": [dict(t=sA["t_total"], L_art=mA["L_art"], L_pince=mA["L_pince"])]}
exemples = {}
for nom, kw in [("RRT", dict(etoile=False, iters=4000)),
                ("RRT + raccourcis", dict(etoile=False, iters=4000)),
                ("RRT* (4000 itérations)", dict(etoile=True, iters=4000, garder_iterations=True))]:
    print(nom, "...")
    res, ok = [], 0
    for g in range(10):
        r = RRT(P, graine=g, **kw)
        try:
            ch, info = r.plan(qa, qb)
        except RuntimeError:
            continue
        if "raccourcis" in nom:
            ch = r.raccourcis(ch)
        ok += 1
        m = mesures(ch)
        res.append(dict(t=info["t"], noeuds=info["noeuds"], **{k: m[k] for k in ("L_art", "L_pince")}))
        if g == 0:
            exemples[nom] = (m["tip"], info)
    resultats[nom] = res
    lignes.append(f"| {nom} | {ok}/10 | {np.mean([r['t'] for r in res]):.1f} s | "
                  f"{np.mean([r['noeuds'] for r in res]):.0f} nœuds | "
                  f"{np.mean([r['L_art'] for r in res]):.2f} ± {np.std([r['L_art'] for r in res]):.2f} | "
                  f"{100 * np.mean([r['L_pince'] for r in res]):.1f} ± {100 * np.std([r['L_pince'] for r in res]):.1f} |")

# --- figure 1 : chemins de la pince ----------------------------------------------------------
fig = plt.figure(figsize=(15, 5.2))
a = fig.add_subplot(1, 2, 1)
o = mod.OBSTACLES[0]
c, h = np.array(o["centre"]), np.array(o["demi_taille"])
a.add_patch(plt.Rectangle((c[1] - h[1], c[0] - h[0]), 2 * h[1], 2 * h[0], color=C[7], alpha=0.6, label="obstacle"))
a.plot(mA["tip"][:, 1], mA["tip"][:, 0], color=C[0], lw=2.2, label="A*")
for i, (nom, (tip, info)) in enumerate(exemples.items()):
    a.plot(tip[:, 1], tip[:, 0], color=C[i + 1], lw=1.6, label=nom)
a.plot(0, 0, "ks", ms=7)
a.invert_xaxis()
a.set_aspect("equal")
a.set_xlabel("y (m)")
a.set_ylabel("x (m)")
a.set_title("Chemin de la pince vu de dessus (graine 0)")
a.legend(fontsize=8, loc="lower left")
a3 = fig.add_subplot(1, 2, 2, projection="3d")
tip, info = exemples["RRT"]
Q, par = info["arbre"]
T = np.array([P.fk(q)[0] for q in Q])
for k in range(1, len(Q)):
    a3.plot(*zip(T[par[k]], T[k]), color=GRIS, lw=0.4, alpha=0.6)
a3.plot(tip[:, 0], tip[:, 1], tip[:, 2], color=C[1], lw=2.5)
a3.plot([c[0]] * 2, [c[1]] * 2, [0, 0.3], color=C[7], lw=6, alpha=0.7)
a3.set_title(f"Arbre RRT ({len(Q)} nœuds) vu par la pince")
a3.set_xlabel("x")
a3.set_ylabel("y")
a3.set_zlabel("z")
sauver(fig, "bonus", "b1_rrt_vs_astar_chemins")

# --- figure 2 : statistiques ---------------------------------------------------------------------
noms = list(resultats)
fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
for a, cle, titre in zip(ax, ("t", "L_art", "L_pince"),
                         ("temps de planification (s)", "longueur articulaire (rad)", "distance parcourue par la pince (m)")):
    data = [[r[cle] for r in resultats[n]] for n in noms]
    bp = a.boxplot(data, patch_artist=True, widths=0.5)
    for i, b in enumerate(bp["boxes"]):
        b.set_facecolor(C[i])
        b.set_alpha(0.75)
    a.set_xticks(range(1, len(noms) + 1), [n.replace(" (4000 itérations)", "") for n in noms], fontsize=8.5)
    a.set_title(titre)
fig.suptitle("A* (déterministe) contre RRT / RRT* (10 graines) sur le même transfert", fontweight="bold")
sauver(fig, "bonus", "b1_rrt_vs_astar_stats")

with open(os.path.join(RACINE, "figures", "bonus", "b1_rrt.md"), "w") as f:
    f.write("\n".join(lignes) + "\n")
print("\n".join(lignes))
