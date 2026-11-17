# PID + IK : trajectoire lisse envoyee au vrai SO100.
# Lancer depuis la racine de lerobot-kinematics, avec le .venv active :
#     python examples/pid_ik_real.py
#
# Principe :
#   waypoint cartesien --IK--> angles de consigne --PID (modele simule)--> trajectoire q(t)
#   q(t) est envoyee au bras en consignes de position (follower_arm.action).
#
# ATTENTION : le PID ne lit PAS la position reelle du bras ici. Il met en forme
# la trajectoire ; c'est l'asservissement interne des servos qui la suit.

import time
import numpy as np
from lerobot_kinematics import lerobot_IK, lerobot_FK, get_robot, feetech_arm

# ---------------------------------------------------------------- reglages
PORT = "COM12"                                 # <- A ADAPTER (Gestionnaire de peripheriques)
CALIB = "examples/main_follower.json"

DT_CMD = 0.02                                  # periode d'envoi des consignes (50 Hz)
SUBSTEPS = 4                                   # pas de simulation par envoi
HOLD = 3.0                                     # duree de chaque palier [s]

# Deplacements successifs (x avant, y, z haut) en metres, par rapport a la pose de depart
WAYPOINTS = [
    (0.03, 0.0, 0.00),
    (0.03, 0.0, 0.02),
    (0.00, 0.0, 0.02),
    (0.00, 0.0, 0.00),                         # retour au depart
]

# Gains PID et modele par articulation (memes valeurs que la simulation)
KP, KI, KD = 2.0, 0.5, 0.15
J = np.array([0.05, 0.04, 0.02, 0.01])
B = np.array([0.20, 0.15, 0.08, 0.04])

# Meme pose et memes limites que lerobot_keycon_gpos_real.py
init_qpos = np.array([0.0, -2.8, 2.8, 0.0, -1.57, -0.157])
lim_lo = np.array([-2.1, -3.1, -0.0, -1.375, -1.57, -0.15])
lim_hi = np.array([2.1, 0.0, 3.1, 1.475, 3.1, 1.5])
lim_lo = np.minimum(lim_lo, init_qpos)         # la pose de depart doit rester autorisee
lim_hi = np.maximum(lim_hi, init_qpos)

MAX_IK_JUMP = 1.0                              # rad : cible refusee si l'IK saute plus loin

# ---------------------------------------------------------------- PID
class PID:
    def __init__(self, kp, ki, kd, dt, out_limit=1.5, i_limit=0.5):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.out_limit, self.i_limit = out_limit, i_limit
        self.integral, self.prev_err = 0.0, None

    def step(self, target, measured):
        err = target - measured
        self.integral = np.clip(self.integral + err * self.dt, -self.i_limit, self.i_limit)
        deriv = 0.0 if self.prev_err is None else (err - self.prev_err) / self.dt
        self.prev_err = err
        u = self.kp * err + self.ki * self.integral + self.kd * deriv
        return np.clip(u, -self.out_limit, self.out_limit)


def solve_ik(q_seed, gpos_target, robot):
    """IK iteree (4 articulations pilotees). Renvoie None si pas de solution."""
    q_ref = q_seed.copy()
    for _ in range(10):
        q_try, _ok = lerobot_IK(q_ref, gpos_target, robot=robot)
        q_try = np.array(q_try, dtype=float)[:4]
        if np.any(q_try == -1.0):
            return None
        q_ref = q_try
        g = np.array(lerobot_FK(q_ref, robot=robot), dtype=float)
        if np.linalg.norm(g[[0, 1, 2]] - gpos_target[[0, 1, 2]]) < 1e-3:
            break
    return q_ref


def send(follower_arm, pan, q4, grip):
    qpos = np.concatenate(([pan], q4, [grip]))
    qpos = np.clip(qpos, lim_lo, lim_hi)
    follower_arm.action(qpos)


# ---------------------------------------------------------------- programme
robot = get_robot('so100')
follower_arm = feetech_arm(driver_port=PORT, calibration_file=CALIB)

pan, grip = init_qpos[0], init_qpos[5]
q = init_qpos[1:5].copy()                      # etat simule des 4 articulations
dq = np.zeros(4)
pids = [PID(KP, KI, KD, DT_CMD / SUBSTEPS) for _ in range(4)]
gpos0 = np.array(lerobot_FK(q, robot=robot), dtype=float)

try:
    print("Envoi de la pose de depart (le bras doit deja etre replie)...")
    t0 = time.time()
    while time.time() - t0 < 2.0:
        send(follower_arm, pan, q, grip)
        time.sleep(DT_CMD)

    for i, (dx, dy, dz) in enumerate(WAYPOINTS):
        target = gpos0.copy()
        target[0] += dx
        target[1] += dy
        target[2] += dz

        q_ref = solve_ik(q, target, robot)
        if q_ref is None or np.max(np.abs(q_ref - q)) > MAX_IK_JUMP:
            print(f"Waypoint {i} refuse (IK sans solution ou saut trop grand).")
            continue
        print(f"Waypoint {i} : cible {np.round([dx, dy, dz], 3)} m -> angles {np.round(q_ref, 3)}")

        t_wp = time.time()
        while time.time() - t_wp < HOLD:
            step_start = time.time()
            for _ in range(SUBSTEPS):
                tau = np.array([pids[k].step(q_ref[k], q[k]) for k in range(4)])
                dq = dq + (tau - B * dq) / J * (DT_CMD / SUBSTEPS)
                q = q + dq * (DT_CMD / SUBSTEPS)
            send(follower_arm, pan, q, grip)
            wait = DT_CMD - (time.time() - step_start)
            if wait > 0:
                time.sleep(wait)

    print("Termine.")
except KeyboardInterrupt:
    print("Interrompu par l'utilisateur.")
finally:
    follower_arm.disconnect()