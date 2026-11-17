# -*- coding: utf-8 -*-
"""
dmp.py — Dynamic Movement Primitives (DMP discrètes, Ijspeert et al. 2013) pour le SO-101.

Idée : apprendre la FORME d'un geste démontré (à la main, couple coupé) puis le rejouer vers un
autre but, plus vite / plus lentement, ou en contournant un obstacle — sans replanifier.

Pour chaque dimension (articulation ou coordonnée de la pince) :
    τ z' = α_z (β_z (g − y) − z) + f(x)        système ressort-amortisseur critique (β_z = α_z/4)
    τ y' = z
    τ x' = −α_x x                              « horloge » qui décroît de 1 vers 0
    f(x) = Σ ψ_i(x) w_i / Σ ψ_i(x) · x (g − y0)    terme de forçage appris (gaussiennes ψ_i)
f s'éteint avec x : la convergence vers g est garantie, quelle que soit la démonstration.
Couplage d'évitement (Hoffmann et al. 2009) en option : terme qui fait tourner la vitesse
autour d'un obstacle ponctuel.

Modes (lancer depuis la racine de lerobot-kinematics, script copié dans examples/) :
    MODE = "teach" : bras couple coupé, on enregistre la démonstration (examples/demo_dmp.json)
    MODE = "sim"   : apprentissage + généralisation vers DECALAGE_BUT, affichage MuJoCo
    MODE = "real"  : idem + envoi au vrai bras (suivi LQR comme les autres scripts)
"""
import json
import os
import sys
import time

import numpy as np


class DMP:
    def __init__(self, n_dims, n_bases=30, alpha_z=25.0, alpha_x=4.0):
        self.n, self.nb = n_dims, n_bases
        self.az, self.bz, self.ax = alpha_z, alpha_z / 4.0, alpha_x
        self.c = np.exp(-alpha_x * np.linspace(0, 1, n_bases))          # centres en x
        self.h = 1.0 / np.diff(self.c) ** 2
        self.h = np.append(self.h, self.h[-1])
        self.w = np.zeros((n_dims, n_bases))

    def _psi(self, x):
        return np.exp(-self.h * (x - self.c) ** 2)

    def apprendre(self, Y, dt):
        """Y : démonstration (T x n) échantillonnée à dt. Régression pondérée locale."""
        Y = np.asarray(Y, float)
        self.tau = (len(Y) - 1) * dt
        self.y0, self.g = Y[0].copy(), Y[-1].copy()
        dY = np.gradient(Y, dt, axis=0)
        ddY = np.gradient(dY, dt, axis=0)
        t = np.arange(len(Y)) * dt
        x = np.exp(-self.ax * t / self.tau)
        f_cible = self.tau ** 2 * ddY - self.az * (self.bz * (self.g - Y) - self.tau * dY)
        Psi = np.array([self._psi(xi) for xi in x])                       # T x nb
        for d in range(self.n):
            s = x * (self.g[d] - self.y0[d])
            for i in range(self.nb):
                num = np.sum(s * Psi[:, i] * f_cible[:, d])
                den = np.sum(s * s * Psi[:, i]) + 1e-10
                self.w[d, i] = num / den
        return self

    def rejouer(self, y0=None, g=None, tau=None, dt=0.01, obstacles=(), gamma=1000.0, beta=3.0,
                duree_extra=0.3):
        """Génère une trajectoire (T x n). obstacles : liste de points (pour n = 3 uniquement)."""
        y0 = self.y0 if y0 is None else np.asarray(y0, float)
        g = self.g if g is None else np.asarray(g, float)
        tau = self.tau if tau is None else tau
        y, z, x = y0.copy(), np.zeros(self.n), 1.0
        out = [y.copy()]
        for _ in range(int((tau * (1 + duree_extra)) / dt)):
            psi = self._psi(x)
            f = (self.w @ psi) / (psi.sum() + 1e-10) * x * (g - y0)
            couplage = np.zeros(self.n)
            v = z / tau
            for o in obstacles:                     # Hoffmann et al. 2009 (rotation autour de o)
                o = np.asarray(o, float)
                if np.linalg.norm(v) < 1e-6:
                    continue
                d = o - y
                r = np.cross(d, v)
                if np.linalg.norm(r) < 1e-9:
                    r = np.cross(d, np.array([0, 0, 1.0]))
                r /= np.linalg.norm(r)
                theta = np.arccos(np.clip(d @ v / (np.linalg.norm(d) * np.linalg.norm(v)), -1, 1))
                Rv = v * np.cos(np.pi / 2) + np.cross(r, v) * np.sin(np.pi / 2) + r * (r @ v) * (1 - np.cos(np.pi / 2))
                couplage += gamma * Rv * theta * np.exp(-beta * theta)
            dz = (self.az * (self.bz * (g - y) - z) + f + couplage) / tau
            y = y + z / tau * dt
            z = z + dz * dt
            x = x - self.ax * x / tau * dt
            out.append(y.copy())
        return np.array(out)


# =============================================================================================
# Script pour le bras (teach / sim / real)
# =============================================================================================
MODE = "sim"
PORT = "COM12"
CALIB_FILE = "examples/main_follower.json"
DEMO_FILE = "examples/demo_dmp.json"
XML_PATH = "examples/scene_so101.xml"
DUREE_DEMO = 5.0            # s d'enregistrement en mode teach
DT = 0.02
DECALAGE_BUT = [0.0, 0.25, 0.0, 0.0]   # le geste est rejoué vers but + décalage (rad, 4 articulations)
VITESSE = 1.0               # > 1 : plus rapide (τ divisé)


def demo_synthetique():
    """Démonstration par défaut : « prendre » (descente en arc puis remontée) en articulaire."""
    t = np.linspace(0, 1, 151)
    q = np.zeros((len(t), 4))
    q[:, 0] = -0.4 + 0.8 * (3 * t ** 2 - 2 * t ** 3)                 # rotation de la base
    q[:, 1] = -0.2 + 0.6 * np.sin(np.pi * t) ** 2 * (t < 0.5) + 0.6 * (t >= 0.5) * (1 - 0.5 * (t - 0.5))
    q[:, 2] = 0.8 - 0.3 * np.sin(np.pi * t)
    q[:, 3] = -0.9 - 0.4 * np.sin(np.pi * t)
    return q, 3.0 / (len(t) - 1)


def main():
    os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    if MODE == "teach":
        from lerobot_kinematics import feetech_arm
        arm = feetech_arm(driver_port=PORT, calibration_file=CALIB_FILE)
        arm.arm_hardware.write("Torque_Enable", 0)
        input("Bras libre : appuie sur Entrée puis montre le geste pendant %.0f s..." % DUREE_DEMO)
        Q, t0 = [], time.time()
        while time.time() - t0 < DUREE_DEMO:
            Q.append(np.array(arm.feedback(), float).tolist())
            time.sleep(DT)
        arm.disconnect()
        with open(DEMO_FILE, "w") as f:
            json.dump(dict(dt=DT, q=Q), f)
        print(f"{len(Q)} échantillons enregistrés dans {DEMO_FILE}")
        return None
    if os.path.exists(DEMO_FILE):
        with open(DEMO_FILE) as f:
            dd = json.load(f)
        Q6 = np.array(dd["q"])
        Qd, dt_demo = Q6[:, :4], dd["dt"]
    else:
        Qd, dt_demo = demo_synthetique()
        Q6 = np.hstack([Qd, np.tile([1.52, 0.0], (len(Qd), 1))])
    dmp = DMP(4).apprendre(Qd, dt_demo)
    g2 = dmp.g + np.array(DECALAGE_BUT)
    Qr = dmp.rejouer(g=g2, tau=dmp.tau / VITESSE, dt=DT)
    print(f"DMP : démonstration {len(Qd)} points ({dmp.tau:.1f} s) -> rejouée vers {np.round(g2, 2)}")
    if MODE == "real":
        # la DMP fournit déjà une trajectoire lisse et datée : envoi direct à 50 Hz aux servos
        from lerobot_kinematics import feetech_arm
        arm = feetech_arm(driver_port=PORT, calibration_file=CALIB_FILE)
        q0 = np.array(arm.feedback(), float)
        if np.max(np.abs(q0[:4] - Qr[0])) > 0.15:
            arm.disconnect()
            raise RuntimeError("Place d'abord le bras près du début de la démonstration (écart > 0.15 rad)")
        arm.arm_hardware.write("Goal_Position", np.rad2deg(q0) * np.array([-1, -1, 1, 1, -1, 1]))
        arm.arm_hardware.write("Torque_Enable", 1)
        try:
            for q4 in Qr:
                t0 = time.time()
                arm.action(np.concatenate([q4, Q6[0, 4:6]]))
                time.sleep(max(0.0, DT - (time.time() - t0)))
        finally:
            arm.disconnect()
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 4, figsize=(14, 3))
    for k in range(4):
        ax[k].plot(np.arange(len(Qd)) * dt_demo, Qd[:, k], "--", label="démonstration")
        ax[k].plot(np.arange(len(Qr)) * DT, Qr[:, k], label="DMP rejouée")
        ax[k].set_title(f"articulation {k + 1}")
    ax[0].legend()
    plt.tight_layout()
    plt.show()
    return dict(demo=Qd, rejeu=Qr, dmp=dmp)


if __name__ == "__main__":
    sys.path.insert(0, os.path.join(os.getcwd(), "examples"))
    main()
