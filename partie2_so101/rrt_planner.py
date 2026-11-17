# -*- coding: utf-8 -*-
"""
rrt_planner.py — RRT et RRT* dans l'espace ARTICULAIRE du SO-101 (aller plus loin, Q8).

Même modèle de collision que astar_planner.ArmPlanner (géométries MuJoCo + boîtes) :
une configuration q est valide si aucune pièce du bras ne touche un obstacle et si le bout de
pince reste hors des obstacles gonflés et au-dessus de la table.

Différence de fond avec A* :
  - A* discrétise l'espace CARTÉSIEN de la pince (grille 1 cm) et résout l'IK case par case ;
    il est complet et optimal à la résolution près, mais le nombre de cases explose avec la
    dimension et la finesse de la grille ;
  - RRT échantillonne directement l'espace des CONFIGURATIONS (4 articulations) : pas d'IK, pas
    de grille, probabilistiquement complet, mais chemin non optimal et aléatoire ;
  - RRT* ajoute le « rewiring » : asymptotiquement optimal (le coût diminue avec le nombre
    d'itérations).

Utilisation :
    from rrt_planner import RRT
    rrt = RRT(planner, etoile=True)          # planner = ArmPlanner(...)
    chemin, info = rrt.plan(q_depart, q_but)
"""
import time

import numpy as np

JOINTS = [0, 1, 2, 3]


class RRT:
    def __init__(self, planner, etoile=False, pas=0.15, pas_verif=0.03, biais_but=0.1, iters=3000,
                 rayon=0.6, graine=0, garder_iterations=False):
        self.P = planner
        self.etoile, self.pas, self.pas_verif = etoile, pas, pas_verif
        self.biais, self.iters, self.rayon = biais_but, iters, rayon
        self.rng = np.random.default_rng(graine)
        self.garder = garder_iterations       # RRT* : continuer après avoir trouvé le but
        self.lo, self.hi = planner.lo[JOINTS], planner.hi[JOINTS]

    # ------------------------------------------------------------------ validité
    def valide(self, q):
        p, _ = self.P.fk(q)
        return self.P.point_free(p) and not self.P.collides(q)

    def arete_valide(self, qa, qb):
        n = max(2, int(np.ceil(np.max(np.abs(qb - qa)[JOINTS]) / self.pas_verif)) + 1)
        for f in np.linspace(0, 1, n)[1:]:
            if not self.valide((1 - f) * qa + f * qb):
                return False
        return True

    # ------------------------------------------------------------------ planification
    def plan(self, q0, q1):
        t0 = time.time()
        q0, q1 = np.array(q0, float), np.array(q1, float)
        noeuds, parent, cout = [q0], [-1], [0.0]
        but_idx, n_verif = None, 0
        for it in range(self.iters):
            if self.rng.random() < self.biais:
                qs = q1.copy()
            else:
                qs = q0.copy()
                qs[JOINTS] = self.rng.uniform(self.lo, self.hi)
            Q = np.array(noeuds)
            d = np.linalg.norm(Q[:, JOINTS] - qs[JOINTS], axis=1)
            i_proche = int(np.argmin(d))
            qn = noeuds[i_proche]
            direction = qs - qn
            dist = np.linalg.norm(direction[JOINTS])
            qnew = qs if dist <= self.pas else qn + direction * (self.pas / dist)
            qnew[4:] = q0[4:]
            n_verif += 1
            if not self.arete_valide(qn, qnew):
                continue
            # --- choix du parent (RRT*) ---
            p_best, c_best = i_proche, cout[i_proche] + np.linalg.norm((qnew - qn)[JOINTS])
            voisins = []
            if self.etoile:
                dn = np.linalg.norm(Q[:, JOINTS] - qnew[JOINTS], axis=1)
                voisins = list(np.where(dn < self.rayon)[0])
                for v in voisins:
                    c = cout[v] + dn[v]
                    if c < c_best and self.arete_valide(noeuds[v], qnew):
                        p_best, c_best = v, c
            noeuds.append(qnew)
            parent.append(p_best)
            cout.append(c_best)
            k = len(noeuds) - 1
            # --- recâblage (RRT*) ---
            if self.etoile:
                for v in voisins:
                    c = c_best + np.linalg.norm((noeuds[v] - qnew)[JOINTS])
                    if c < cout[v] and self.arete_valide(qnew, noeuds[v]):
                        parent[v], cout[v] = k, c
            if np.linalg.norm((qnew - q1)[JOINTS]) < 1e-9:
                if but_idx is None or c_best < cout[but_idx]:
                    but_idx = k
                if not (self.etoile and self.garder):
                    break
        if but_idx is None:
            raise RuntimeError(f"RRT : but non atteint en {self.iters} itérations")
        chemin = [but_idx]
        while parent[chemin[-1]] != -1:
            chemin.append(parent[chemin[-1]])
        chemin = [noeuds[i] for i in reversed(chemin)]
        info = dict(algo="RRT*" if self.etoile else "RRT", noeuds=len(noeuds), iterations=it + 1,
                    t=time.time() - t0, cout=float(cout[but_idx]), arbre=(np.array(noeuds), np.array(parent)))
        return np.array(chemin), info

    def raccourcis(self, chemin, essais=150):
        """Lissage aléatoire : on remplace un sous-chemin par une ligne droite articulaire valide."""
        ch = [np.array(q) for q in chemin]
        for _ in range(essais):
            if len(ch) < 3:
                break
            i, j = sorted(self.rng.choice(len(ch), 2, replace=False))
            if j - i < 2:
                continue
            if self.arete_valide(ch[i], ch[j]):
                ch = ch[:i + 1] + ch[j:]
        return np.array(ch)


def densifier(chemin, pas=0.02):
    """Ré-échantillonne un chemin articulaire (pour la loi horaire et la visualisation)."""
    out = [chemin[0]]
    for a, b in zip(chemin[:-1], chemin[1:]):
        n = max(1, int(np.ceil(np.max(np.abs(b - a)) / pas)))
        out += [a + (b - a) * f for f in np.linspace(0, 1, n + 1)[1:]]
    return np.array(out)
