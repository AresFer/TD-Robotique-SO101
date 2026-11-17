# -*- coding: utf-8 -*-
"""
astar_planner.py — planificateur A* (grille 3D) avec vérification de collision MuJoCo, SO-101.

!! RECONSTRUCTION !!  Le fichier original importé par pick_place_astar.py n'a pas été versionné.
Ce module ré-implémente EXACTEMENT l'interface utilisée par pick_place_astar.py :

    ArmPlanner(xml, obstacles, table_z=, res=, margin=, clearance=)
    .m, .qadr                       modèle MuJoCo (avec les obstacles) et indices qpos
    ._context(q_from, q_to)         fixe rotation du poignet / pince pour l'IK
    .fk(q)                -> (p, a) position du bout de pince + axe de la pince (site gripperframe)
    .ik(p, a, seed, w_axis, iters)  -> (q, ok)     IK moindres carrés amortis (Jacobien MuJoCo)
    .ik_robust(p, a, seed)          -> (q, ok)     IK avec plusieurs graines
    .collides(q)          -> bool   une pièce du bras touche-t-elle un obstacle ?
    .joint_path(points, q_from, q_to) -> chemin articulaire n x 6 le long d'une polyligne
    .plan(q0, q1, heuristic=True)   -> (chemin n x 6, points lissés)   A* (ou Dijkstra)

Principe (conforme à la docstring de pick_place_astar.py) :
  1. la pince se déplace dans une grille 3D de pas `res` ; une case est libre si
       - le bout de pince est hors des obstacles gonflés de `margin`,
       - il est au moins `clearance` au-dessus de la table,
       - l'IK (position seule, mouvement articulaire minimal) trouve une configuration en
         partant de celle de la case parente (ou, à défaut, des configurations extrêmes),
       - AUCUNE pièce du bras (géométries de collision MuJoCo) n'est à moins de `arm_margin`
         d'un obstacle (mj_geomDistance) ;
     les cases sont évaluées paresseusement, seulement quand A* les atteint ;
  2. A* : coût = longueur euclidienne, heuristique = distance euclidienne au but (admissible
     et consistante -> chemin optimal sur la grille) ; heuristic=False -> Dijkstra ;
  3. lissage par raccourcis : on relie le point courant au point le plus lointain joignable
     en ligne droite (segment entièrement valide, IK continue tous les res/2) ; les
     configurations ainsi validées forment directement le chemin articulaire ;
  4. raccord final (orientation, rotation du poignet, pince) par interpolation articulaire
     vérifiée contre les collisions.
"""
import heapq
import time

import numpy as np
import mujoco

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
SITE = "gripperframe"
IK_JOINTS = [0, 1, 2, 3]          # pan, épaule, coude, flexion poignet (rotation poignet et pince fixées)


class ArmPlanner:
    def __init__(self, xml_path, obstacles, table_z=0.0, res=0.01, margin=0.025, clearance=0.03,
                 arm_margin=0.006, path_step=0.005, verbose=True):
        self.obstacles = [dict(centre=np.asarray(o["centre"], float),
                               demi=np.asarray(o["demi_taille"], float)) for o in obstacles]
        self.table_z, self.res, self.margin = float(table_z), float(res), float(margin)
        self.clearance, self.arm_margin, self.path_step = float(clearance), float(arm_margin), path_step
        self.verbose = verbose
        self.m = self._build_model(xml_path)
        self.d = mujoco.MjData(self.m)
        self.qadr = np.array([self.m.jnt_qposadr[self.m.joint(n).id] for n in JOINT_NAMES])
        self.dadr = np.array([self.m.jnt_dofadr[self.m.joint(n).id] for n in JOINT_NAMES])
        self.lo = self.m.jnt_range[[self.m.joint(n).id for n in JOINT_NAMES], 0]
        self.hi = self.m.jnt_range[[self.m.joint(n).id for n in JOINT_NAMES], 1]
        self.site = self.m.site(SITE).id
        base = self.m.body("base").id
        self.robot_geoms = [g for g in range(self.m.ngeom)
                            if self.m.geom_group[g] == 3 and self.m.geom_bodyid[g] != base]
        self.obs_geoms = [self.m.geom(f"obstacle_{i}").id for i in range(len(self.obstacles))]
        self._fromto = np.zeros(6)
        self._roll, self._g0, self._g1 = 0.0, 1.0, 1.0
        self.stats = []                      # statistiques de chaque planification

    # ------------------------------------------------------------------ modèle + obstacles
    def _build_model(self, xml_path):
        spec = mujoco.MjSpec.from_file(xml_path)
        for i, o in enumerate(self.obstacles):
            g = spec.worldbody.add_geom()
            g.name = f"obstacle_{i}"
            g.type = mujoco.mjtGeom.mjGEOM_BOX
            g.size = o["demi"]
            g.pos = o["centre"]
            g.rgba = [0.85, 0.15, 0.15, 0.55]
            g.contype, g.conaffinity, g.group = 0, 0, 1   # visuel ; la distance est calculée à la main
        return spec.compile()

    # ------------------------------------------------------------------ cinématique
    def _context(self, q_from, q_to):
        """Rotation du poignet et ouverture de pince utilisées par l'IK (non optimisées)."""
        self._roll = float(q_from[4])
        self._g0, self._g1 = float(q_from[5]), float(q_to[5])

    def _set(self, q):
        self.d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)

    def fk(self, q):
        self._set(q)
        p = self.d.site_xpos[self.site].copy()
        a = self.d.site_xmat[self.site].reshape(3, 3)[:, 0].copy()   # axe x du site = axe de la pince
        return p, a

    def ik(self, p, a, seed, w_axis=1.0, iters=100, tol=1e-3, lam=0.02, axis_strict=True, q_post=None,
           k_post=0.3):
        """IK moindres carrés amortis à priorités : 1) position du bout de pince, 2) axe de la pince,
        (option) 3) posture de référence q_post dans le noyau de la tâche de position."""
        q = np.array(seed, float)
        q[4] = self._roll
        p, a = np.asarray(p, float), np.asarray(a, float)
        a = a / (np.linalg.norm(a) + 1e-12)
        jp, jr = np.zeros((3, self.m.nv)), np.zeros((3, self.m.nv))
        cols = self.dadr[IK_JOINTS]
        for _ in range(iters):
            self._set(q)
            pc = self.d.site_xpos[self.site]
            ac = self.d.site_xmat[self.site].reshape(3, 3)[:, 0]
            ep = p - pc
            er = np.cross(ac, a)
            if np.linalg.norm(ep) < tol and (w_axis == 0 or np.linalg.norm(er) < 0.02) and (
                    q_post is None or np.max(np.abs(q_post[IK_JOINTS] - q[IK_JOINTS])) < 0.01 or _ > 6):
                break
            mujoco.mj_jacSite(self.m, self.d, jp, jr, self.site)
            Jp, Jr = jp[:, cols], jr[:, cols]
            # tâche 1 (prioritaire) : position ; tâche 2 : axe de la pince dans le noyau de Jp
            Jp_pinv = Jp.T @ np.linalg.inv(Jp @ Jp.T + lam ** 2 * np.eye(3))
            dq = Jp_pinv @ ep
            if w_axis > 0:
                N = np.eye(len(cols)) - Jp_pinv @ Jp
                JrN = Jr @ N
                dq2 = JrN.T @ np.linalg.solve(JrN @ JrN.T + lam ** 2 * np.eye(3), w_axis * er - Jr @ dq)
                dq = dq + N @ dq2
            if q_post is not None:
                N = np.eye(len(cols)) - Jp_pinv @ Jp
                dq = dq + N @ (k_post * (q_post[IK_JOINTS] - q[IK_JOINTS]))
            dq = np.clip(dq, -0.2, 0.2)
            q[IK_JOINTS] = np.clip(q[IK_JOINTS] + dq, self.lo[IK_JOINTS], self.hi[IK_JOINTS])
        pc, ac = self.fk(q)
        ok = np.linalg.norm(p - pc) < 3 * tol and (w_axis == 0 or not axis_strict or np.dot(ac, a) > 0.95)
        return q, bool(ok)

    def ik_robust(self, p, a, seed, w_axis=1.0):
        best, err_best = None, np.inf
        seeds = [np.array(seed, float)]
        rng = np.random.default_rng(0)
        for _ in range(8):
            s = np.array(seed, float)
            s[IK_JOINTS] += rng.normal(0, 0.3, len(IK_JOINTS))
            seeds.append(s)
        for s in seeds:
            q, ok = self.ik(p, a, s, w_axis=w_axis, iters=200)
            if ok and not self.collides(q):
                return q, True
            err = np.linalg.norm(self.fk(q)[0] - p)
            if err < err_best:
                best, err_best = q, err
        return best, False

    # ------------------------------------------------------------------ collisions
    def collides(self, q):
        """True si une géométrie de collision du bras est à moins de arm_margin d'un obstacle."""
        if not self.obs_geoms:
            return False
        self._set(q)
        for go in self.obs_geoms:
            for gr in self.robot_geoms:
                dist = mujoco.mj_geomDistance(self.m, self.d, gr, go, self.arm_margin, self._fromto)
                if dist < self.arm_margin:
                    return True
        return False

    def point_free(self, p, near_end=False):
        """Le bout de pince seul : hors des obstacles gonflés et au-dessus de la table."""
        if not near_end and p[2] < self.table_z + self.clearance - 1e-9:
            return False
        for o in self.obstacles:
            if np.all(np.abs(p - o["centre"]) <= o["demi"] + self.margin):
                return False
        return True

    # ------------------------------------------------------------------ chemin articulaire
    def joint_path(self, points, q_from, q_to, step=None, w_axis=1.0, strict=True):
        """Configurations tous les `step` m le long de la polyligne `points` (IK à chaud).
        strict=True : l'axe de la pince suit l'interpolation de l'axe de départ vers celui
        d'arrivée (montées / descentes verticales). Lève RuntimeError si l'IK échoue ou si le
        bras touche un obstacle."""
        step = step or self.path_step
        pts = [np.asarray(p, float) for p in points]
        seg = [np.linalg.norm(b - a) for a, b in zip(pts[:-1], pts[1:])]
        L = float(np.sum(seg))
        _, a0 = self.fk(q_from)
        _, a1 = self.fk(q_to)
        n = max(2, int(np.ceil(L / step)) + 1)
        s_all = np.linspace(0, L, n)
        cum = np.concatenate([[0], np.cumsum(seg)])
        path, q = [np.array(q_from, float)], np.array(q_from, float)
        for s in s_all[1:-1]:
            k = min(np.searchsorted(cum, s, side="right") - 1, len(seg) - 1)
            f = 0 if seg[k] < 1e-12 else (s - cum[k]) / seg[k]
            p = pts[k] + f * (pts[k + 1] - pts[k])
            r = s / L
            a = (1 - r) * a0 + r * a1
            q_prev = q.copy()
            sols = []
            for g in ([q_prev] if strict else [q_prev, np.array(q_to, float), np.array(q_from, float)]):
                qi, ok = self.ik(p, a, g, w_axis=w_axis, iters=80, tol=1e-3 if strict else 2e-3,
                                 axis_strict=strict)
                if ok:
                    sols.append(qi)
                    if np.max(np.abs(qi - q_prev)[:4]) < 0.2:
                        break
            if not sols:
                raise RuntimeError(f"IK impossible en {np.round(p, 3)}")
            q = min(sols, key=lambda qi: np.max(np.abs(qi - q_prev)[:4]))
            q[5] = (1 - r) * self._g0 + r * self._g1
            saut = np.max(np.abs(q - q_prev)[:4])
            if saut > 0.2:          # changement de branche de l'IK : passage articulaire vérifié
                for f in np.linspace(0, 1, int(np.ceil(saut / 0.05)) + 1)[1:-1]:
                    qi = (1 - f) * q_prev + f * q
                    if self.collides(qi):
                        raise RuntimeError(f"collision pendant un changement de branche en {np.round(p, 3)}")
                    path.append(qi)
            if self.collides(q):
                raise RuntimeError(f"collision du bras en {np.round(p, 3)}")
            path.append(q.copy())
        # raccord final : si l'orientation a dérivé, réorientation articulaire vérifiée
        q_to = np.array(q_to, float)
        ecart = np.max(np.abs(path[-1] - q_to))
        if ecart > 0.05:
            for f in np.linspace(0, 1, int(np.ceil(ecart / 0.02)) + 1)[1:-1]:
                qi = (1 - f) * path[-1] + f * q_to
                if self.collides(qi):
                    raise RuntimeError("collision pendant la réorientation finale")
                path.append(qi)
        path.append(q_to)
        return np.array(path)

    # ------------------------------------------------------------------ A*
    def _cell(self, p):
        return tuple(np.round((np.asarray(p) - self.origin) / self.res).astype(int))

    def _center(self, c):
        return self.origin + np.array(c) * self.res

    def _valid(self, c, q_seed, cache, a_ref):
        if c in cache:
            return cache[c]
        p = self._center(c)
        q = None
        if self.point_free(p):
            r = np.hypot(p[0], p[1])
            if 0.08 < r < 0.45 and p[2] < self.table_z + 0.40:
                # graine = configuration de la case parente ; si l'IK échoue on essaie aussi les
                # configurations d'arrivée et de départ (poses très repliées, proches des butées)
                graines = [q_seed] + [qe for _, qe in self._ends]
                q = None
                for g in graines:
                    qi, ok = self.ik(p, a_ref, g, w_axis=0.0, iters=40, tol=2e-3, q_post=self._posture(p) if self._use_posture else None)
                    if ok and not self.collides(qi):
                        q = qi
                        break
        cache[c] = q
        return q

    def _segment(self, p0, p1, q0, a_ref):
        """Configurations (pas res/2) le long du segment p0 -> p1 depuis q0, ou None si invalide."""
        n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / (self.res / 2))))
        for f in np.linspace(0, 1, n)[1:]:
            if not self.point_free(p0 + f * (p1 - p0)):
                return None
        q, qs = np.array(q0, float), []
        for f in np.linspace(0, 1, n)[1:]:
            p = p0 + f * (p1 - p0)
            q_prev = q
            q, ok = self.ik(p, a_ref, q, w_axis=0.0, iters=40, tol=2e-3, q_post=self._posture(p) if self._use_posture else None)
            if not ok or self.collides(q) or np.max(np.abs(q - q_prev)[:4]) > 0.2:
                return None
            qs.append(q.copy())
        return qs

    def _posture(self, p):
        """Posture de référence : mélange des configurations de départ et d'arrivée selon la
        proximité de p à chaque extrémité -> le bras « glisse » d'une posture à l'autre et
        arrive au but dans la même branche d'IK que q1 (pas de retournement brutal)."""
        (_, qa), (_, qb) = self._ends
        da, db = np.linalg.norm(p - self._pa), np.linalg.norm(p - self._pb)
        f = da / (da + db + 1e-9)
        return (1 - f) * qa + f * qb

    def _interp_libre(self, qa, qb, pas=0.05):
        """Interpolation articulaire qa -> qb vérifiée contre les collisions (sans qa)."""
        ecart = np.max(np.abs(np.asarray(qb) - qa))
        out = []
        for f in np.linspace(0, 1, max(2, int(np.ceil(ecart / pas)) + 1))[1:]:
            qi = (1 - f) * np.asarray(qa) + f * np.asarray(qb)
            if self.collides(qi):
                raise RuntimeError("collision pendant une interpolation articulaire")
            out.append(qi)
        return out

    def plan(self, q0, q1, heuristic=True, max_expand=200000):
        """A* (heuristic=True) ou Dijkstra (heuristic=False) de la pince entre fk(q0) et fk(q1).
        1er essai : IK « mouvement minimal » ; si le raccord à q1 est impossible (changement de
        branche d'IK près d'un obstacle), 2e essai avec une posture de référence interpolée."""
        try:
            return self._plan(q0, q1, heuristic, max_expand, posture=False)
        except RuntimeError as e:
            if self.verbose:
                print(f"    ({e}) -> nouvel essai avec posture de référence")
            return self._plan(q0, q1, heuristic, max_expand, posture=True)

    def _plan(self, q0, q1, heuristic, max_expand, posture):
        self._context(q0, q1)
        t0 = time.time()
        p0, a0 = self.fk(q0)
        p1, a1 = self.fk(q1)
        a_ref = (a0 + a1) / 2
        self.origin = p0 - np.round(p0 / self.res) * self.res   # grille alignée sur le départ
        start, goal = self._cell(p0), self._cell(p1)
        cache = {start: np.array(q0, float), goal: np.array(q1, float)}
        self._ends = [(start, np.array(q0, float)), (goal, np.array(q1, float))]
        self._pa, self._pb = p0, p1
        self._use_posture = posture
        g = {start: 0.0}
        parent = {start: None}
        h = (lambda c: self.res * np.linalg.norm(np.subtract(c, goal))) if heuristic else (lambda c: 0.0)
        openl = [(h(start), 0.0, start)]
        closed = set()
        dirs = [(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1) if (i, j, k) != (0, 0, 0)]
        n_exp = 0
        while openl:
            _, gc, c = heapq.heappop(openl)
            if c in closed:
                continue
            closed.add(c)
            n_exp += 1
            if c == goal or n_exp > max_expand:
                break
            for d in dirs:
                nb = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
                if nb in closed:
                    continue
                if self._valid(nb, cache[c], cache, a_ref) is None:
                    continue
                ng = gc + self.res * np.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
                if ng < g.get(nb, np.inf):
                    g[nb], parent[nb] = ng, c
                    heapq.heappush(openl, (ng + h(nb), ng, nb))
        if goal not in parent:
            raise RuntimeError("A* : aucun chemin trouvé (obstacle infranchissable ?)")
        cells = [goal]
        while parent[cells[-1]] is not None:
            cells.append(parent[cells[-1]])
        cells.reverse()
        pts = [p0] + [self._center(c) for c in cells[1:-1]] + [p1]
        qs = [cache[c] for c in cells]
        t_search = time.time() - t0
        # --- lissage par raccourcis : on relie le point courant au point le plus lointain
        #     joignable en ligne droite ; les configurations validées sont conservées ---
        i, smooth, qpath = 0, [pts[0]], [np.array(q0, float)]
        while i < len(pts) - 1:
            j = len(pts) - 1
            seg = self._segment(pts[i], pts[j], qpath[-1], a_ref)
            while seg is None and j > i + 1:
                j -= 1
                seg = self._segment(pts[i], pts[j], qpath[-1], a_ref)
            if seg is None:                       # case voisine : passage articulaire vérifié
                seg = self._interp_libre(qpath[-1], qs[j])
            qpath += seg
            smooth.append(pts[j])
            i = j
        # raccord final vers q1 (orientation / rotation du poignet / pince)
        if np.max(np.abs(qpath[-1] - q1)) > 1e-6:
            qpath += self._interp_libre(qpath[-1], np.array(q1, float))
        path = np.array(qpath)
        frac = np.linspace(0, 1, len(path))
        path[:, 5] = (1 - frac) * q0[5] + frac * q1[5]
        L_grid = float(np.sum(np.linalg.norm(np.diff(np.array(pts), axis=0), axis=1)))
        L_smooth = float(np.sum(np.linalg.norm(np.diff(np.array(smooth), axis=0), axis=1)))
        st = dict(algo="A*" if heuristic else "Dijkstra", posture=posture, expanded=n_exp, evaluated=len(cache),
                  t_search=t_search, t_total=time.time() - t0, L_grid=L_grid, L_smooth=L_smooth,
                  n_cells=len(cells), n_waypoints=len(smooth))
        self.stats.append(st)
        if self.verbose:
            print(f"    {st['algo']:8s}: {n_exp:6d} cases développées, {len(cache):6d} évaluées, "
                  f"{t_search:5.2f} s, chemin {100 * L_grid:.1f} cm -> lissé {100 * L_smooth:.1f} cm "
                  f"({len(smooth)} points)")
        return path, [np.asarray(p) for p in smooth[1:-1]]
