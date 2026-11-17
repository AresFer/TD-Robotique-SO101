# PID articulaire pilote par la cinematique inverse (IK), verifie par la FK.
# Lancer depuis la racine de lerobot-kinematics, avec le .venv active :
#     python examples/pid_ik_sim.py
#
# Principe :
#   cible cartesienne --IK--> angles de consigne --PID--> couple --modele--> angles
#   angles --FK--> position de l'effecteur (pour mesurer l'erreur reelle)
#
# Le "modele" ici est un simple modele par articulation (inertie + frottement),
# c'est une HYPOTHESE de simulation, pas la dynamique reelle du SO100.

import numpy as np
import matplotlib.pyplot as plt
from lerobot_kinematics import lerobot_IK, lerobot_FK, get_robot

robot = get_robot('so100')

# ---------------------------------------------------------------- PID
class PID:
    def __init__(self, kp, ki, kd, dt, out_limit=None, i_limit=None):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.out_limit, self.i_limit = out_limit, i_limit
        self.integral = 0.0
        self.prev_err = None

    def step(self, target, measured):
        err = target - measured
        self.integral += err * self.dt
        if self.i_limit is not None:                      # anti-windup
            self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)
        deriv = 0.0 if self.prev_err is None else (err - self.prev_err) / self.dt
        self.prev_err = err
        u = self.kp * err + self.ki * self.integral + self.kd * deriv
        if self.out_limit is not None:                    # saturation
            u = np.clip(u, -self.out_limit, self.out_limit)
        return u

# ---------------------------------------------------------------- parametres
dt = 0.005
T = 4.0
n = int(T / dt)

# Articulations pilotees par l'IK : Pitch, Elbow, Wrist_Pitch, Wrist_Roll
# (meme convention que init_qpos[1:5] dans lerobot_keycon_gpos_real.py)
q = np.array([-3.14, 3.14, 0.0, -1.57])
dq = np.zeros(4)

# Modele par articulation : J*ddq = tau - b*dq   (a ajuster)
J = np.array([0.05, 0.04, 0.02, 0.01])
b = np.array([0.20, 0.15, 0.08, 0.04])

# Gains PID (a regler)
kp, ki, kd = 2.0, 0.5, 0.15
pids = [PID(kp, ki, kd, dt, out_limit=1.5, i_limit=0.5) for _ in range(4)]

# ---------------------------------------------------------------- cible
gpos0 = np.array(lerobot_FK(q, robot=robot), dtype=float)
gpos_target = gpos0.copy()
gpos_target[0] += 0.02      # +2 cm vers l'avant
gpos_target[2] += 0.02      # +2 cm vers le haut

q_ref, ok = lerobot_IK(q, gpos_target, robot=robot)
q_ref = np.array(q_ref, dtype=float)[:4]
gpos_ik = np.array(lerobot_FK(q_ref, robot=robot), dtype=float)
print("IK success :", ok)
print("Erreur IK seule [mm] :", np.linalg.norm(gpos_ik[[0, 1, 2]] - gpos_target[[0, 1, 2]]) * 1000)
if np.any(q_ref == -1.0):
    raise SystemExit("IK sans solution pour cette cible : reduis le deplacement.")
if np.max(np.abs(q_ref - q)) > 1.0:
    raise SystemExit("Consigne IK trop eloignee de la pose actuelle (> 1 rad) : cible refusee.")
print("Angles de consigne (IK) :", np.round(q_ref, 3))

# ---------------------------------------------------------------- simulation
t_hist, q_hist, err_hist = [], [], []
for i in range(n):
    tau = np.array([pids[k].step(q_ref[k], q[k]) for k in range(4)])
    ddq = (tau - b * dq) / J
    dq = dq + ddq * dt
    q = q + dq * dt

    gpos = np.array(lerobot_FK(q, robot=robot), dtype=float)
    err = np.linalg.norm(gpos[[0, 1, 2]] - gpos_target[[0, 1, 2]])

    t_hist.append(i * dt)
    q_hist.append(q.copy())
    err_hist.append(err)

q_hist = np.array(q_hist)
print("Erreur articulaire finale [rad] :", np.round(q_ref - q_hist[-1], 4))
print(f"Erreur finale de position de l'effecteur : {err_hist[-1] * 1000:.2f} mm")

# ---------------------------------------------------------------- graphiques
names = ["Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]
fig, ax = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
for k in range(4):
    ax[0].plot(t_hist, q_hist[:, k], label=names[k])
    ax[0].axhline(q_ref[k], ls=":", lw=0.8)
ax[0].set_ylabel("angle [rad]")
ax[0].legend()
ax[0].set_title("Reponse PID articulaire (pointilles = consigne IK)")
ax[1].plot(t_hist, np.array(err_hist) * 1000)
ax[1].set_ylabel("erreur effecteur [mm]")
ax[1].set_xlabel("temps [s]")
plt.tight_layout()
plt.show()
