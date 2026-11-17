# -*- coding: utf-8 -*-
"""
Q8 (évitement d'obstacle / zone interdite) — script ORIGINAL partie2_so101/pick_place_astar.py

1. Jumeau numérique, points par défaut (+ GIF).
2. Scénario de l'essai réel reconstruit (points A/B lus sur le graphe réel) et hypothèse de la
   boucle réelle plus lente que DT : balayage du temps de communication du bus série.
3. A* contre Dijkstra (même chemin, coût de recherche différent).
4. Zones interdites : marge de sécurité, deux obstacles, « mur » -> chemins vus de dessus.
Lancer :  python simulations/s6_astar_obstacles.py         (≈ 10 min)
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from outils.banc import (executer, ecrire_points, effacer_points, charger, QA_REEL, QB_REEL,  # noqa: E402
                         RACINE, EX)
from outils.style import plt, sauver, C, GRIS  # noqa: E402

FIG = os.path.join(RACINE, "figures", "partie2")
GIF = os.path.join(RACINE, "figures", "gif")
ETAPES = sys.argv[1:] or ["1", "2", "4"]     # ex. : python simulations/s6_astar_obstacles.py 4

def etape1(md):
    print("=== 1. jumeau, points par défaut ===")
    effacer_points()
    mod, lg = executer("pick_place_astar", "real", "p2_q8_astar_jumeau", FIG,
                       gif=os.path.join(GIF, "q8_astar_jumeau.gif"),
                       reglages=dict(COMPARE_DIJKSTRA=True),
                       kw_vue=dict(tous_les=4, largeur=420, hauteur=315))
    # les statistiques sont stockées dans l'objet planner du script : on relance la planification
    P = astar_planner.ArmPlanner(mod.XML_PATH, mod.OBSTACLES, table_z=0.02, res=mod.GRID_RES,
                                 margin=mod.MARGIN, clearance=mod.CLEARANCE, verbose=False)
    qA, qB = mod.load_points(P)
    qA_up, qB_up = mod.above(P, qA, mod.APPROACH_DZ), mod.above(P, qB, mod.APPROACH_DZ)
    qAc, qBc = qA_up.copy(), qB_up.copy()
    qAc[5] = qBc[5] = 0.0
    for h in (True, False):
        P.plan(qAc, qBc, heuristic=h)
    sA, sD = P.stats[-2], P.stats[-1]
    md.append("| Planificateur | cases développées | cases évaluées (IK + collisions) | temps de recherche | "
              "longueur grille | longueur lissée |")
    md.append("|---|---|---|---|---|---|")
    for s in (sA, sD):
        md.append(f"| {s['algo']} | {s['expanded']} | {s['evaluated']} | {s['t_search']:.1f} s | "
                  f"{100 * s['L_grid']:.1f} cm | {100 * s['L_smooth']:.1f} cm |")
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
    for a, cle, titre in zip(ax, ("expanded", "evaluated", "t_search"),
                             ("cases développées", "cases évaluées (IK + collision)", "temps de recherche (s)")):
        vals = [sA[cle], sD[cle]]
        a.bar(["A*", "Dijkstra"], vals, color=[C[0], C[1]], width=0.55)
        for i, v in enumerate(vals):
            a.text(i, v, f"{v:.1f}" if cle == "t_search" else f"{v:d}", ha="center", va="bottom", fontsize=9)
        a.set_title(titre)
    fig.suptitle(f"Q8 — A* contre Dijkstra sur le transfert A → B (longueur sur la grille : A* "
                 f"{100 * sA['L_grid']:.1f} cm, Dijkstra {100 * sD['L_grid']:.1f} cm)", fontweight="bold", y=1.04)
    sauver(fig, "partie2", "p2_q8_astar_vs_dijkstra")



def etape2(md):
    print("=== 2. scénario réel reconstruit ===")
    ecrire_points(QA_REEL, QB_REEL)
    res = []
    for tl in [0.0, 0.006, 0.010, 0.014, 0.018, 0.022]:
        pref = f"p2_q8_reel_reconstruit_boucle{round(1000 * max(0.02, 0.004 + 2 * tl))}ms" if tl in (0.0, 0.018) else ""
        mod, lg = executer("pick_place_astar", "real", pref, FIG if pref else None,
                           kw_jumeau=dict(t_ecriture=0.004 if tl else 0.0, t_lecture=tl), kw_vue=dict(actif=False))
        j = lg["jumeau"]
        per = np.mean(j.periodes)
        retard = np.abs(lg["q"][:, :5] - lg["ref"][:, :5]).max()
        mes = np.abs(lg["meas"][:, :5] - lg["q"][:, :5]).max()
        res.append((tl, per, lg["t"][-1], retard, mes))
        print(f"   t_lecture {1000 * tl:.0f} ms -> période {1000 * per:.1f} ms, durée {lg['t'][-1]:.1f} s")
    res = np.array(res)
    T_REEL = 35.5   # durée lue sur figures/reel/pick_place_astar_lqr_reel.jpg
    per_estimee = np.interp(T_REEL, res[:, 2], res[:, 1])
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
    ax[0].plot(1000 * res[:, 1], res[:, 2], "o-", color=C[0])
    ax[0].axhline(T_REEL, color=C[7], ls="--", lw=1.2)
    ax[0].text(1000 * res[0, 1], T_REEL + 1, f"durée de l'essai réel ≈ {T_REEL} s", color=C[7], fontsize=8.5)
    ax[0].axvline(1000 * per_estimee, color=GRIS, ls=":", lw=1)
    ax[0].set_xlabel("période réelle de la boucle (ms)   [DT du script = 20 ms]")
    ax[0].set_ylabel("durée totale du pick & place (s)")
    ax[0].set_title(f"La durée réelle correspond à une boucle de ≈ {1000 * per_estimee:.0f} ms")
    ax[1].plot(1000 * res[:, 1], res[:, 3], "o-", color=C[1], label="max |état LQR − consigne|")
    ax[1].plot(1000 * res[:, 1], res[:, 4], "o-", color=C[2], label="max |mesure − état LQR|")
    ax[1].set_xlabel("période réelle de la boucle (ms)")
    ax[1].set_ylabel("rad")
    ax[1].set_title("Boucle lente : l'état LQR traîne derrière la consigne")
    ax[1].legend()
    fig.suptitle("Q8 — Jumeau numérique : effet du temps de communication avec les servos (scénario réel reconstruit)",
                 fontweight="bold", y=1.04)
    sauver(fig, "partie2", "p2_q8_periode_boucle")
    md.append("")
    md.append("| t_lecture simulé | période de boucle | durée totale | max\\|état − consigne\\| | max\\|mesure − état\\| |")
    md.append("|---|---|---|---|---|")
    for tl, per, T, r1, r2 in res:
        md.append(f"| {1000 * tl:.0f} ms | {1000 * per:.1f} ms | {T:.1f} s | {r1:.3f} rad | {r2:.3f} rad |")
    md.append(f"\nPériode de boucle compatible avec la durée de l'essai réel ({T_REEL} s) : ≈ {1000 * per_estimee:.0f} ms")

    # comparaison visuelle réel / jumeau
    import matplotlib.image as mpimg
    fig, ax = plt.subplots(1, 2, figsize=(16, 6.2))
    ax[0].imshow(mpimg.imread(os.path.join(RACINE, "figures", "reel", "pick_place_astar_lqr_reel.jpg")))
    ax[0].set_title("Essai RÉEL (vrai SO-101)")
    ax[1].imshow(mpimg.imread(os.path.join(FIG, "p2_q8_reel_reconstruit_boucle40ms_1.png")))
    ax[1].set_title("Jumeau numérique : mêmes A/B, boucle de 40 ms")
    for a in ax:
        a.axis("off")
        a.grid(False)
    sauver(fig, "partie2", "p2_q8_reel_vs_jumeau")



def etape4(md):
    print("=== 4. zones interdites ===")
    effacer_points()
    mod = charger("pick_place_astar")
    variantes = [("marge 1 cm", [dict(centre=[0.25, 0.05, 0.05], demi_taille=[0.02, 0.02, 1])], 0.01),
                 ("marge 2.5 cm (réglage du TD)", [dict(centre=[0.25, 0.05, 0.05], demi_taille=[0.02, 0.02, 1])], 0.025),
                 ("marge 5 cm", [dict(centre=[0.25, 0.05, 0.05], demi_taille=[0.02, 0.02, 1])], 0.05),
                 ("poteau + plot (h = 14 cm)", [dict(centre=[0.25, 0.05, 0.05], demi_taille=[0.02, 0.02, 1]),
                                                dict(centre=[0.18, 0.02, 0.08], demi_taille=[0.015, 0.015, 0.06])],
                  0.025),
                 ("mur bas (h = 12 cm)", [dict(centre=[0.24, 0.02, 0.06], demi_taille=[0.10, 0.012, 0.06])], 0.025)]
    fig, ax = plt.subplots(1, len(variantes), figsize=(4 * len(variantes), 4.4), sharey=True)
    md.append("\n| Zone interdite | longueur du chemin lissé | cases développées | temps | hauteur max de la pince |")
    md.append("|---|---|---|---|---|")
    for a, (nom, obs, marge) in zip(ax, variantes):
        P = astar_planner.ArmPlanner(mod.XML_PATH, obs, table_z=0.02, res=0.01, margin=marge, clearance=0.03,
                                     verbose=False)
        qA, qB = mod.load_points(P)
        try:
            qa, qb = mod.above(P, qA, 0.06), mod.above(P, qB, 0.06)
            path, wp = P.plan(qa, qb)
            tip = np.array([P.fk(q)[0] for q in path])
            s = P.stats[-1]
            a.plot(tip[:, 1], tip[:, 0], color=C[0], lw=2)
            md.append(f"| {nom} | {100 * s['L_smooth']:.1f} cm | {s['expanded']} | {s['t_search']:.1f} s | "
                      f"{100 * tip[:, 2].max():.1f} cm |")
            a.set_title(f"{nom}\n{100 * s['L_smooth']:.0f} cm, z max {100 * tip[:, 2].max():.0f} cm", fontsize=9.5)
        except Exception as e:
            md.append(f"| {nom} | échec : {e} | | | |")
            a.set_title(f"{nom}\néchec", fontsize=9.5)
        for o in obs:
            c, h = np.array(o["centre"]), np.array(o["demi_taille"])
            a.add_patch(plt.Rectangle((c[1] - h[1], c[0] - h[0]), 2 * h[1], 2 * h[0], color=C[7], alpha=0.55))
            a.add_patch(plt.Rectangle((c[1] - h[1] - marge, c[0] - h[0] - marge), 2 * (h[1] + marge), 2 * (h[0] + marge),
                                      fill=False, ls="--", ec=C[7], lw=1))
        pa, pb = P.fk(qA)[0], P.fk(qB)[0]
        a.plot([pa[1], pb[1]], [pa[0], pb[0]], "*", color=C[3], ms=13)
        a.plot(0, 0, "ks", ms=7)
        a.invert_xaxis()
        a.set_aspect("equal")
        a.set_xlabel("y (m)")
    ax[0].set_ylabel("x (m)")
    fig.suptitle("Q8 — Interdire une zone : chemin de la pince (vue de dessus) ; rouge = obstacle, pointillé = marge",
                 fontweight="bold", y=1.06)
    sauver(fig, "partie2", "p2_q8_zones_interdites")



if __name__ == "__main__":
    charger("pick_place_astar")
    import astar_planner  # noqa: E402,F401  (dans EX, chargé par le script)
    for e, f in (("1", etape1), ("2", etape2), ("4", etape4)):
        if e in ETAPES:
            md = []
            f(md)
            with open(os.path.join(FIG, f"p2_q8_astar_etape{e}.md"), "w") as fh:
                fh.write("\n".join(md) + "\n")
            print("\n".join(md))
