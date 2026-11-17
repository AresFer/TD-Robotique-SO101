# -*- coding: utf-8 -*-
"""
Aller plus loin : deux défauts trouvés dans nos scripts LQR et leur correctif, démontrés en simulation.

1. Pics de commande u = ±15 rad/s² (visibles sur le graphe RÉEL) : la consigne est une
   interpolation LINÉAIRE par morceaux du chemin articulaire ; son accélération, calculée par
   différences finies (h = 10 ms), est une suite d'impulsions à chaque point du chemin, et la
   durée T est calculée sur la longueur du chemin sans tenir compte de ses virages
   (q'' = q_s''·ṡ² + q_s'·s̈ : la courbure du chemin ajoute de l'accélération).
   Correctif : spline cubique du chemin (C²), dérivées analytiques, et durée recalée pour que
   chaque articulation respecte ACC_MAX et VEL_MAX.
2. Boucle plus lente que DT : lqr_step intègre avec DT = 20 ms alors que la consigne avance avec
   l'horloge murale -> l'état LQR traîne. Correctif : intégrer avec le dt MESURÉ.
Lancer :  python simulations/s9_ameliorations.py
"""
import os
import sys

import numpy as np
from scipy.interpolate import CubicSpline

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from outils.banc import charger, effacer_points, RACINE, EX  # noqa: E402
from outils.style import plt, sauver, C, GRIS  # noqa: E402
from outils.jumeau import BrasFeetechSimule  # noqa: E402

effacer_points()
mod = charger("pick_place_astar")
import astar_planner  # noqa: E402

P = astar_planner.ArmPlanner(mod.XML_PATH, mod.OBSTACLES, table_z=0.02, verbose=False)
qA, qB = mod.load_points(P)
qa, qb = mod.above(P, qA, 0.06), mod.above(P, qB, 0.06)
qa[5] = qb[5] = 0.0
chemin, _ = P.plan(qa, qb)
K = mod.build_lqr()[2]


class ReferenceLisse(mod.Reference):
    """Même loi horaire que Reference, mais chemin interpolé par spline cubique (C²)
    et dérivées ANALYTIQUES (plus de différences finies)."""

    def __init__(self, path):
        super().__init__(path)
        cum, keep = self.cum, np.concatenate([[True], np.diff(self.cum) > 1e-9])
        self.sp = CubicSpline(cum[keep], self.path[keep], bc_type="clamped")
        # durée recalée pour que l'accélération ET la vitesse de CHAQUE articulation restent sous
        # ACC_MAX / VEL_MAX (multiplier T par k divise les vitesses par k et les accélérations par k²)
        tt = np.linspace(0, self.T, 400)
        _, dq, ddq = zip(*[self(t) for t in tt])
        k_acc = np.sqrt(np.max(np.abs(ddq)) / mod.ACC_MAX)
        k_vit = np.max(np.abs(dq)) / mod.VEL_MAX
        self.T_initial = self.T
        self.T *= max(1.0, k_acc, k_vit)

    def __call__(self, t, h=None):
        tau = np.clip(t / self.T, 0, 1)
        s = (3 * tau ** 2 - 2 * tau ** 3) * self.D
        ds = (6 * tau - 6 * tau ** 2) * self.D / self.T * (0 < t / self.T < 1)
        dds = (6 - 12 * tau) * self.D / self.T ** 2 * (0 < t / self.T < 1)
        return self.sp(s), self.sp(s, 1) * ds, self.sp(s, 2) * ds ** 2 + self.sp(s, 1) * dds


def boucle(ref, periode=0.02, dt_integ=None, jumeau=False):
    """Boucle de pick_place_astar.py (sans affichage) ; dt_integ=None -> DT fixe du script."""
    x = np.concatenate([chemin[0], np.zeros(6)])
    bras = BrasFeetechSimule(os.path.join(EX, "scene_so101.xml"), q0=chemin[0], dt_boucle=periode) if jumeau else None
    T, Xs, Us, Rs, Ms = [], [], [], [], []
    t = 0.0
    while t < ref.T + 1.0:
        q_r, dq_r, a_r = ref(t)
        if dt_integ is None:
            x, u = mod.lqr_step(x, np.concatenate([q_r, dq_r]), K, a_r)
        else:                                          # correctif : intégration avec le dt réel
            u = np.clip(K @ (np.concatenate([q_r, dq_r]) - x) + a_r, -1.5 * mod.ACC_MAX, 1.5 * mod.ACC_MAX)
            dq = np.clip(x[6:] + u * dt_integ, -1.5 * mod.VEL_MAX, 1.5 * mod.VEL_MAX)
            x = np.concatenate([x[:6] + dq * dt_integ, dq])
        T.append(t)
        Xs.append(x[:6].copy())
        Us.append(u.copy())
        Rs.append(q_r.copy())
        if bras:
            Ms.append(bras.action(x[:6].copy()))
        t += periode
    return dict(t=np.array(T), x=np.array(Xs), u=np.array(Us), r=np.array(Rs), m=np.array(Ms) if Ms else None)


r1, r2 = mod.Reference(chemin), ReferenceLisse(chemin)
b1, b2 = boucle(r1, jumeau=True), boucle(r2, jumeau=True)
md = ["| Variante | durée | max\\|u\\| (rad/s²) | ∫u² dt | max\\|état − consigne\\| (rad) | écart pince au chemin planifié (mm, moy / max) | configurations de l'état en collision | max\\|mesure − état\\| jumeau (rad) |",
      "|---|---|---|---|---|---|---|---|"]
tip_plan = np.array([P.fk(q)[0] for q in chemin])


def ecart_chemin(X):
    tp = np.array([P.fk(q)[0] for q in X])
    d = np.array([np.min(np.linalg.norm(tip_plan - p, axis=1)) for p in tp])
    return 1000 * d.mean(), 1000 * d.max()


fig, ax = plt.subplots(2, 2, figsize=(14, 7.5))
for i, (nom, b) in enumerate([("interpolation linéaire + différences finies (scripts actuels)", b1),
                              (f"spline + durée recalée (correctif) : T = {r2.T:.2f} s au lieu de {r2.T_initial:.2f} s", b2)]):
    for j in range(4):
        ax[0, i].plot(b["t"], b["u"][:, j], color=C[j], lw=1.1, label=mod.JOINT_NAMES[j])
    ax[0, i].set_title(nom, fontsize=10)
    ax[0, i].set_ylim(-16, 16)
    ax[0, i].set_ylabel("u = q'' (rad/s²)")
    em, eM = ecart_chemin(b["x"])
    n_coll = sum(P.collides(q) for q in b["x"])
    md.append(f"| {nom} | {b['t'][-1]:.1f} s | {np.abs(b['u']).max():.1f} | {np.sum(b['u'] ** 2) * 0.02:.0f} | "
              f"{np.abs(b['x'] - b['r']).max():.3f} | {em:.1f} / {eM:.1f} | {n_coll} / {len(b['x'])} | "
              f"{np.abs(b['m'][:, :5] - b['x'][:, :5]).max():.3f} |")
ax[0, 0].legend(fontsize=8)
for i, b in enumerate([b1, b2]):
    tp = np.array([P.fk(q)[0] for q in b["x"]])
    ax[1, 0].plot(tp[:, 1], tp[:, 0], color=C[i], label=["actuel", "spline"][i])
ax[1, 0].plot(tip_plan[:, 1], tip_plan[:, 0], "--", color=GRIS, label="chemin planifié (A*)")
ax[1, 0].invert_xaxis()
ax[1, 0].set_aspect("equal")
ax[1, 0].set_title("Pince (état LQR) vue de dessus")
ax[1, 0].legend(fontsize=8)
ax[1, 0].set_xlabel("y (m)")
ax[1, 0].set_ylabel("x (m)")

# --- 2. période de boucle --------------------------------------------------------------------
per = np.array([0.02, 0.025, 0.03, 0.035, 0.04, 0.05])
retard_fixe, retard_corr = [], []
for p in per:
    bf = boucle(r2, periode=p)
    bc = boucle(r2, periode=p, dt_integ=p)
    retard_fixe.append(np.abs(bf["x"] - bf["r"]).max())
    retard_corr.append(np.abs(bc["x"] - bc["r"]).max())
ax[1, 1].plot(1000 * per, retard_fixe, "o-", color=C[1], label="lqr_step avec DT = 20 ms fixe (actuel)")
ax[1, 1].plot(1000 * per, retard_corr, "o-", color=C[2], label="intégration avec le dt mesuré (correctif)")
ax[1, 1].set_xlabel("période réelle de la boucle (ms)")
ax[1, 1].set_ylabel("max |état LQR − consigne| (rad)")
ax[1, 1].set_title("Boucle plus lente que prévu")
ax[1, 1].legend(fontsize=8)
fig.suptitle("Deux améliorations des scripts LQR, vérifiées en simulation (transfert A* au-dessus de A → B)",
             fontweight="bold")
sauver(fig, "bonus", "b3_ameliorations_lqr")
md.append("")
md.append("| Période de boucle | retard max (DT fixe) | retard max (dt mesuré) |")
md.append("|---|---|---|")
for p, a, b in zip(per, retard_fixe, retard_corr):
    md.append(f"| {1000 * p:.0f} ms | {a:.3f} rad | {b:.3f} rad |")
with open(os.path.join(RACINE, "figures", "bonus", "b3_ameliorations.md"), "w") as f:
    f.write("\n".join(md) + "\n")
print("\n".join(md))
