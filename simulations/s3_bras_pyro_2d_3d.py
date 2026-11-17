# -*- coding: utf-8 -*-
"""
Q5 : « Testez des modèles de bras 2D avant de passer à la 3D, testez des modèles multi-DDL »
avec la bibliothèque pyro (modèles dynamiques H(q) q'' + C q' + g(q) + d = B u).

1. Bras plan 2 DDL (pyro TwoLinkManipulator) : cible cartésienne -> IK (Newton sur J(q))
   -> trois contrôleurs comparés :
      a. PID articulaire (pyro JointPID)
      b. LQR sur le modèle LINÉARISÉ autour de la cible (pyro linearize + Riccati)
      c. couple calculé + LQR : u = H(q)(v) + C q' + g(q), v = -K x  => q'' = v (double
         intégrateur : exactement le modèle x' = A x + B u utilisé pour le SO-101)
2. Influence de Q / R sur le couple calculé + LQR.
3. Bras plan 5 DDL (redondant) : contrôleur cinématique de l'effecteur (pyro).
4. Bras 3 DDL en 3D (pyro ThreeLinkManipulator3D) : PID d'effecteur.
5. Étoile dessinée par le bras 2 DDL (IK + LQR) -> GIF.
Lancer :  python simulations/s3_bras_pyro_2d_3d.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from outils.style import plt, sauver, C, GRIS, RACINE  # noqa: E402
from pyro.dynamic import manipulator  # noqa: E402
from pyro.dynamic.statespace import linearize, StateSpaceSystem  # noqa: E402
from pyro.control import robotcontrollers  # noqa: E402
from pyro.control.lqr import synthesize_lqr_controller  # noqa: E402
from pyro.analysis.costfunction import QuadraticCostFunction  # noqa: E402

md = []
rob = manipulator.TwoLinkManipulator()


def ik_newton(robot, r_d, q0, iters=100):
    q = np.array(q0, float)
    for _ in range(iters):
        e = r_d - robot.forward_kinematic_effector(q)
        if np.linalg.norm(e) < 1e-9:
            break
        J = robot.J(q)
        q += np.linalg.pinv(J) @ e
    return q


def simuler(robot, loi, x0, T=4.0, dt=0.001):
    x = np.array(x0, float)
    X, U = [], []
    for k in range(int(T / dt)):
        u = loi(x, k * dt)
        X.append(x.copy())
        U.append(u)
        k1 = robot.f(x, u)
        k2 = robot.f(x + dt / 2 * k1, u)
        k3 = robot.f(x + dt / 2 * k2, u)
        k4 = robot.f(x + dt * k3, u)
        x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return np.arange(len(X)) * dt, np.array(X), np.array(U)


def gain_double_integrateur(n, qp, qv, r):
    Z, I = np.zeros((n, n)), np.eye(n)
    ss = StateSpaceSystem(np.block([[Z, I], [Z, Z]]), np.vstack([Z, I]), np.eye(2 * n), np.zeros((2 * n, n)))
    cf = QuadraticCostFunction(2 * n, n)
    cf.Q, cf.R = np.diag([qp] * n + [qv] * n), r * np.eye(n)
    return synthesize_lqr_controller(ss, cf).K


# =========================================================================================
# 1. 2 DDL : PID / LQR linéarisé / couple calculé + LQR
# =========================================================================================
r_d = np.array([0.45, 0.35])
q_d = ik_newton(rob, r_d, [0.3, 0.8])
x0 = np.zeros(4)
print("cible", r_d, "-> IK q_d =", np.round(q_d, 3), " FK(q_d) =", np.round(rob.forward_kinematic_effector(q_d), 4))

# a. PID pyro (gains de l'exemple pyro, ki pour compenser la gravité)
pid = robotcontrollers.JointPID(2)
pid.rbar = q_d
pid.kp, pid.kd, pid.ki = np.array([25, 5]), np.array([5, 2]), np.array([8, 4])
cl = pid + rob
cl.x0 = np.zeros(6)
tr = cl.compute_trajectory(tf=4.0, n=4001)
t_pid, X_pid = tr.t, tr.x[:, :4]
U_pid = tr.u
# b. LQR sur le modèle linéarisé autour de (q_d, 0) avec u_bar = g(q_d)
rob.xbar = np.concatenate([q_d, [0, 0]])
rob.ubar = rob.g(q_d)
ss_lin = linearize(rob, 1e-5)
cf = QuadraticCostFunction(4, 2)
cf.Q, cf.R = np.diag([100, 100, 1, 1]), np.eye(2)
ctl_lin = synthesize_lqr_controller(ss_lin, cf, rob.xbar, rob.ubar)
K_lin = ctl_lin.K
t_l, X_l, U_l = simuler(rob, lambda x, t: rob.ubar - K_lin @ (x - rob.xbar), x0)
# c. couple calculé + LQR double intégrateur
K_ct = gain_double_integrateur(2, 100, 1, 1)


def couple_calcule(K, q_ref):
    def loi(x, t):
        q, dq = x[:2], x[2:]
        v = -K @ np.concatenate([q - q_ref, dq])
        return rob.H(q) @ v + rob.C(q, dq) @ dq + rob.g(q) + rob.d(q, dq)
    return loi


t_c, X_c, U_c = simuler(rob, couple_calcule(K_ct, q_d), x0)
fig, ax = plt.subplots(1, 3, figsize=(16, 4.4))
for i, (nom, t, X, U) in enumerate([("PID articulaire (pyro JointPID)", t_pid, X_pid, U_pid),
                                    ("LQR sur modèle linéarisé", t_l, X_l, U_l),
                                    ("couple calculé + LQR", t_c, X_c, U_c)]):
    ax[0].plot(t, X[:, 0], color=C[i], label=nom)
    ax[0].plot(t, X[:, 1], "--", color=C[i])
    ax[1].plot(t, U[:, 0], color=C[i], label=nom)
    E = np.array([rob.forward_kinematic_effector(q) for q in X[::20, :2]])
    ax[2].plot(E[:, 0], E[:, 1], color=C[i], label=nom)
    err = np.linalg.norm(np.array([rob.forward_kinematic_effector(q) for q in X[-200:, :2]]) - r_d, axis=1).mean()
    hors = np.where(np.linalg.norm(X[:, :2] - q_d, axis=1) > 0.02)[0]
    md.append(f"| {nom} | {1000 * err:.2f} mm | {t[hors[-1]] if len(hors) else 0:.2f} s | "
              f"{np.abs(U).max():.1f} N·m |")
for q in q_d:
    ax[0].axhline(q, color=GRIS, lw=0.8, ls=":")
ax[0].set_title("Angles (— q1, -- q2) ; pointillés = IK de la cible")
ax[0].set_xlabel("t (s)")
ax[0].set_ylabel("rad")
ax[1].set_title("Couple de l'épaule u1")
ax[1].set_xlabel("t (s)")
ax[1].set_ylabel("N·m")
ax[2].plot(*r_d, "*", color=C[7], ms=14, label="cible")
ax[2].plot(0, 0, "ks", ms=6)
ax[2].set_aspect("equal")
ax[2].set_title("Trajectoire de l'effecteur dans le plan")
ax[2].set_xlabel("x (m)")
ax[2].set_ylabel("y (m)")
ax[2].legend(fontsize=8)
ax[0].legend(fontsize=8)
fig.suptitle("Q5 — Bras plan 2 DDL (pyro) : FK/IK + PID vs LQR vs couple calculé + LQR", fontweight="bold")
sauver(fig, "partie2", "p2_q5_bras2ddl_pid_lqr")

# =========================================================================================
# 2. Q/R sur le couple calculé + LQR
# =========================================================================================
fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
for i, (qp, r) in enumerate([(1, 1), (9, 1), (100, 1), (1000, 1), (100, 0.01)]):
    K = gain_double_integrateur(2, qp, 1, r)
    t, X, U = simuler(rob, couple_calcule(K, q_d), x0, T=3.0)
    wn = (qp / r) ** 0.25
    ax[0].plot(t, X[:, 0], color=C[i], label=f"Q={qp:g}, R={r:g}  (ωn={wn:.1f} rad/s, ζ={K[0, 2] / (2 * wn):.2f})")
    ax[1].plot(t, U[:, 0], color=C[i])
ax[0].axhline(q_d[0], color=GRIS, ls=":", lw=1)
ax[0].set_title("q1 : la réponse est celle d'un 2e ordre ωn = (Q/R)^¼")
ax[0].set_xlabel("t (s)")
ax[0].legend(fontsize=7.5)
ax[1].set_title("Couple u1 : plus Q/R est grand, plus le pic de couple est élevé")
ax[1].set_xlabel("t (s)")
ax[1].set_ylabel("N·m")
ax[1].set_ylim(-60, 60)
sauver(fig, "partie2", "p2_q5_bras2ddl_QR")

# =========================================================================================
# 3. Bras plan 5 DDL redondant : contrôleur cinématique pyro
# =========================================================================================
rob5 = manipulator.FiveLinkPlanarManipulator()
vit5 = manipulator.SpeedControlledManipulator.from_manipulator(rob5)
kc = robotcontrollers.EndEffectorKinematicController(vit5, 1)
kc.rbar = np.array([1.0, 1.0])
cl5 = kc + vit5
cl5.x0 = np.array([0.1, 0.1, 0.1, 0.1, 0.1])
tr5 = cl5.compute_trajectory(tf=6.0, n=601)
fig, ax = plt.subplots(1, 2, figsize=(12, 4.6))
for k in range(5):
    ax[0].plot(tr5.t, tr5.x[:, k], color=C[k], label=f"q{k + 1}")
ax[0].set_title("5 DDL : 5 articulations pour 2 coordonnées (x, y)")
ax[0].set_xlabel("t (s)")
ax[0].set_ylabel("rad")
ax[0].legend(fontsize=8)
for j, idx in enumerate([0, 60, 150, 600]):
    lines = rob5.forward_kinematic_lines(tr5.x[idx])[0]
    pts = lines[0]
    ax[1].plot(pts[:, 0], pts[:, 1], "o-", color=C[j], ms=3, label=f"t = {tr5.t[idx]:.1f} s")
E5 = np.array([rob5.forward_kinematic_effector(q) for q in tr5.x])
ax[1].plot(E5[:, 0], E5[:, 1], ":", color=GRIS)
ax[1].plot(*kc.rbar, "*", color=C[7], ms=14)
ax[1].set_aspect("equal")
ax[1].set_title("Posture du bras redondant (pseudo-inverse de J)")
ax[1].legend(fontsize=8)
sauver(fig, "partie2", "p2_q5_bras5ddl_cinematique")
md.append(f"| 5 DDL cinématique : erreur finale | {1000 * np.linalg.norm(E5[-1] - kc.rbar):.2f} mm |  |  |")

# =========================================================================================
# 4. 3 DDL en 3D : PID d'effecteur (pyro)
# =========================================================================================
rob3 = manipulator.ThreeLinkManipulator3D()
epid = robotcontrollers.EndEffectorPID(rob3)
epid.rbar = np.array([0.5, 1.0, 2.0])
epid.kp, epid.kd, epid.ki = np.array([100, 100, 100]), np.array([30, 30, 30]), np.array([90, 90, 90])
cl3 = epid + rob3
cl3.x0 = np.zeros(9)
tr3 = cl3.compute_trajectory(tf=5.0, n=2001)
E3 = np.array([rob3.forward_kinematic_effector(x[:3]) for x in tr3.x])
fig = plt.figure(figsize=(12, 4.6))
a = fig.add_subplot(1, 2, 1)
for k, lab in enumerate("xyz"):
    a.plot(tr3.t, E3[:, k], color=C[k], label=lab)
    a.axhline(epid.rbar[k], color=C[k], ls=":", lw=1)
a.set_title("3 DDL 3D : position de l'effecteur (PID cartésien)")
a.set_xlabel("t (s)")
a.set_ylabel("m")
a.legend()
a3 = fig.add_subplot(1, 2, 2, projection="3d")
a3.plot(E3[:, 0], E3[:, 1], E3[:, 2], color=C[0])
for j, idx in enumerate([0, 300, 2000]):
    pts = rob3.forward_kinematic_lines(tr3.x[idx, :3])[0][0]
    a3.plot(pts[:, 0], pts[:, 1], pts[:, 2], "o-", color=C[j + 1], ms=3)
a3.scatter(*epid.rbar, color=C[7], s=80, marker="*")
a3.set_title("Trajectoire 3D et postures")
sauver(fig, "partie2", "p2_q5_bras3ddl_3d")
md.append(f"| 3 DDL 3D (PID effecteur) : erreur finale | {1000 * np.linalg.norm(E3[-1] - epid.rbar):.2f} mm |  |  |")

# =========================================================================================
# 5. Étoile dessinée par le bras 2 DDL (IK + couple calculé + LQR) -> GIF
# =========================================================================================
c, R = np.array([0.45, 0.2]), 0.18
ang = np.pi / 2 + 2 * np.pi * np.arange(5) / 5
S = [c + R * np.array([np.cos(a), np.sin(a)]) for a in ang]
S = [S[i] for i in [0, 2, 4, 1, 3, 0]]
pts = []
for a_, b_ in zip(S[:-1], S[1:]):
    pts += [a_ + f * (b_ - a_) for f in np.linspace(0, 1, 60, endpoint=False)]
pts.append(S[-1])
qs, q = [], ik_newton(rob, pts[0], [0.3, 0.8])
for p in pts:
    q = ik_newton(rob, p, q)
    qs.append(q.copy())
qs = np.array(qs)
dt_pt = 0.02
K = gain_double_integrateur(2, 400, 1, 1)
x = np.concatenate([qs[0], [0, 0]])
X, dt = [], 0.001
for k in range(int(len(qs) * dt_pt / dt)):
    i = min(int(k * dt / dt_pt), len(qs) - 2)
    f = k * dt / dt_pt - i
    q_ref = (1 - f) * qs[i] + f * qs[i + 1]
    dq_ref = (qs[i + 1] - qs[i]) / dt_pt
    qv, dqv = x[:2], x[2:]
    v = -K @ np.concatenate([qv - q_ref, dqv - dq_ref])
    u = rob.H(qv) @ v + rob.C(qv, dqv) @ dqv + rob.g(qv) + rob.d(qv, dqv)
    k1 = rob.f(x, u)
    x = x + dt * k1
    X.append(x.copy())
X = np.array(X)
E = np.array([rob.forward_kinematic_effector(q) for q in X[::10, :2]])
Sa = np.array(S)
dev = [np.min(np.linalg.norm(np.array(pts) - e, axis=1)) for e in E]
md.append(f"| Étoile 2 DDL : écart moyen / max à la consigne | {1000 * np.mean(dev):.1f} mm / {1000 * np.max(dev):.1f} mm |  |  |")
fig, a = plt.subplots(figsize=(5.5, 5.5))
a.plot(Sa[:, 0], Sa[:, 1], "--", color=GRIS, label="étoile visée")
a.plot(E[:, 0], E[:, 1], color=C[0], label="effecteur (IK + couple calculé + LQR)")
for idx in np.linspace(0, len(X) - 1, 6).astype(int):
    L = rob.forward_kinematic_lines(X[idx, :2])[0][0]
    a.plot(L[:, 0], L[:, 1], "o-", color=C[1], alpha=0.35, ms=3)
a.set_aspect("equal")
a.legend(fontsize=8, loc="lower left")
a.set_title("Bras 2 DDL qui dessine une étoile")
sauver(fig, "partie2", "p2_q5_bras2ddl_etoile")

try:
    from PIL import Image
    images = []
    for idx in np.linspace(0, len(X) - 1, 90).astype(int):
        f, a = plt.subplots(figsize=(3.6, 3.6), dpi=80)
        a.plot(Sa[:, 0], Sa[:, 1], "--", color=GRIS, lw=1)
        Ek = np.array([rob.forward_kinematic_effector(q) for q in X[:idx + 1:10, :2]])
        if len(Ek):
            a.plot(Ek[:, 0], Ek[:, 1], color=C[0], lw=2)
        L = rob.forward_kinematic_lines(X[idx, :2])[0][0]
        a.plot(L[:, 0], L[:, 1], "o-", color=C[1], lw=3, ms=5)
        a.set_xlim(-0.1, 0.8)
        a.set_ylim(-0.2, 0.7)
        a.set_aspect("equal")
        a.set_xticks([])
        a.set_yticks([])
        a.set_title("2 DDL : IK + LQR", fontsize=9)
        f.canvas.draw()
        images.append(Image.fromarray(np.asarray(f.canvas.buffer_rgba())[:, :, :3]).convert("P", palette=Image.ADAPTIVE))
        plt.close(f)
    os.makedirs(os.path.join(RACINE, "figures", "gif"), exist_ok=True)
    images[0].save(os.path.join(RACINE, "figures", "gif", "q5_bras2ddl_etoile.gif"), save_all=True,
                   append_images=images[1:], duration=60, loop=0, optimize=True)
    print("  gif    -> figures/gif/q5_bras2ddl_etoile.gif")
except Exception as e:  # pragma: no cover
    print("GIF non généré :", e)

with open(os.path.join(RACINE, "figures", "partie2", "p2_q5_bras_pyro.md"), "w") as f:
    f.write(f"Cible {r_d.tolist()} -> IK q_d = {np.round(q_d, 4).tolist()}\n\n")
    f.write(f"K (LQR linéarisé) =\n{np.round(K_lin, 2)}\n\n")
    f.write("| Contrôleur | erreur finale effecteur | temps pour entrer à ±0.02 rad | couple max |\n|---|---|---|---|\n")
    f.write("\n".join(md) + "\n")
print("\n".join(md))
