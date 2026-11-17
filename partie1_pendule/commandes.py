# -*- coding: utf-8 -*-
"""
Lois de commande du pendule EDUKIT + simulateur temps discret réaliste.

Trois contrôleurs, tous exécutés à période fixe Ts (bloqueur d'ordre 0) sur le modèle
NON LINÉAIRE intégré finement (RK4, 0.25 ms) :

1. PIDShawn     : transcription fidèle du notebook de Shawn Hymel (Q1).
                  Erreur en degrés, sortie = DÉPLACEMENT du moteur pas-à-pas à chaque pas
                  (« move_deg » envoyé avec CMD_MOVE_BY), intégrale en ms.
2. DoublePID    : architecture du firmware EDUKIT (ENABLE_DUAL_PID) : PID pendule + PID rotor
                  dont les sorties (accélérations) s'additionnent, dérivées filtrées passe-bas.
3. LQR          : retour d'état u = -K (x - x_ref), K calculé par pyro (Riccati) (Q3).

Le simulateur modélise aussi : quantification du codeur (0.15°), bruit de mesure,
retard pur de la boucle, saturation en accélération et en vitesse du moteur, erreur de
calage de la verticale (inclinaison du socle), poussée sur le pendule.
"""
import numpy as np

from modele_pendule import (PARAM_EDUKIT, U_MAX, V_MAX, THETA_MAX, ALPHA_CHUTE,
                            RESOLUTION_CODEUR, matrices_lineaires, rk4)

DEG = np.pi / 180


# =========================================================================================
# Estimation des vitesses (identique pour tous les contrôleurs : comparaison équitable)
# =========================================================================================
class DeriveeFiltree:
    """Dérivée par différence finie + filtre passe-bas du 1er ordre (fréquence de coupure fc)."""

    def __init__(self, fc, Ts):
        w = 2 * np.pi * fc * Ts
        self.a = w / (1 + w)
        self.prev, self.d = None, 0.0
        self.Ts = Ts

    def __call__(self, v):
        if self.prev is not None:
            self.d += self.a * ((v - self.prev) / self.Ts - self.d)
        self.prev = v
        return self.d


# =========================================================================================
# 1. PID de Shawn Hymel (pendulum-pid.ipynb) — commande en DÉPLACEMENT du moteur
# =========================================================================================
class PIDShawn:
    """
    Code original (boucle Python <-> Arduino) :
        error    = SETPOINT - enc_deg                    # degrés
        integral = integral + error * interval           # interval en ms !
        derivative = (error - error_prev) / interval
        move_deg = K_P*error + K_I*integral + K_D*derivative + BIAS
        ctrl.step(CMD_MOVE_BY, [move_deg])               # le rotor AVANCE de move_deg

    Le moteur exécute ce déplacement avec son profil (accélération/vitesse max du L6474).
    Valeurs par défaut du notebook : K_P = 1, K_I = 0.005, K_D = 0.
    """
    nom = "PID Shawn Hymel"
    commande_en_position = True
    resolution_codeur = 2 * np.pi / 1200      # sketch Arduino : ENC_STEPS_PER_ROTATION = 1200 (0.3°)
    micro_pas = 2 * np.pi / 1600              # STEP_MODE_8 : 200 x 8 micro-pas par tour

    def __init__(self, Kp=1.0, Ki=0.005, Kd=0.0, Ts=0.006, bias=0.0):
        self.Kp, self.Ki, self.Kd, self.Ts, self.bias = Kp, Ki, Kd, Ts, bias
        self.integ, self.e_prev = 0.0, 0.0

    def __call__(self, mes, t, ref_theta=0.0):
        e = -mes["alpha"] / DEG                    # SETPOINT - angle (en degrés)
        dt_ms = self.Ts * 1000
        self.integ += e * dt_ms
        deriv = (e - self.e_prev) / dt_ms
        self.e_prev = e
        move_deg = self.Kp * e + self.Ki * self.integ + self.Kd * deriv + self.bias
        # move_stepper_by() : steps = (int)(deg * 200 * 8 / 360) -> troncature au micro-pas
        return np.trunc(move_deg * DEG / self.micro_pas) * self.micro_pas


# =========================================================================================
# 2. Double PID du firmware EDUKIT (sommes des sorties = accélération du rotor)
# =========================================================================================
class DoublePID:
    """
    u = PID_pendule(α) + PID_rotor(θ - θ_ref)
    Gains du firmware (MODE_1, en pas codeur et pas moteur) convertis en unités SI :
        pendule : Kp = 300 -> 225 (rad/s²)/rad ; Kd = 30 -> 22.5 (rad/s²)/(rad/s)
        rotor   : Kp = 15             ; Kd = 7.5   (mêmes unités des deux côtés)
    (1 pas codeur = 2π/2400 rad, 1 pas moteur = 2π/3200 rad -> facteur 2400/3200 = 0.75)
    Signes : le PID rotor est en « rétroaction positive » apparente (il éloigne d'abord le rotor
    de sa consigne) : c'est la signature d'un système à non-minimum de phase.
    """
    nom = "Double PID (EDUKIT)"
    commande_en_position = False

    def __init__(self, Kp_p=225.0, Ki_p=0.0, Kd_p=22.5, Kp_r=15.0, Ki_r=0.0, Kd_r=7.5,
                 Ts=0.002, fc_p=10.0, fc_r=50.0, i_max=1.0):
        self.g = dict(Kp_p=Kp_p, Ki_p=Ki_p, Kd_p=Kd_p, Kp_r=Kp_r, Ki_r=Ki_r, Kd_r=Kd_r)
        self.Ts = Ts
        self.d_al, self.d_th = DeriveeFiltree(fc_p, Ts), DeriveeFiltree(fc_r, Ts)
        self.i_al = self.i_th = 0.0
        self.i_max = i_max

    def __call__(self, mes, t, ref_theta=0.0):
        g = self.g
        al, th = mes["alpha"], mes["theta"] - ref_theta
        dal, dth = self.d_al(mes["alpha"]), self.d_th(mes["theta"])
        self.i_al = np.clip(self.i_al + al * self.Ts, -self.i_max, self.i_max)
        self.i_th = np.clip(self.i_th + th * self.Ts, -self.i_max, self.i_max)
        u_pend = -(g["Kp_p"] * al + g["Ki_p"] * self.i_al + g["Kd_p"] * dal)
        u_rot = +(g["Kp_r"] * th + g["Ki_r"] * self.i_th + g["Kd_r"] * dth)
        return u_pend + u_rot


# =========================================================================================
# 3. LQR (pyro)
# =========================================================================================
def gain_lqr(Q, R, p=PARAM_EDUKIT, avec_pyro=True):
    """Gain K (1x4) minimisant J = ∫ xᵀQx + uᵀRu dt pour le modèle linéarisé."""
    A, B = matrices_lineaires(p)
    Q = np.atleast_2d(np.asarray(Q, float))
    R = np.atleast_2d(np.asarray(R, float))
    if avec_pyro:
        from pyro.dynamic.statespace import StateSpaceSystem
        from pyro.analysis.costfunction import QuadraticCostFunction
        from pyro.control.lqr import synthesize_lqr_controller
        ss = StateSpaceSystem(A, B, np.eye(4), np.zeros((4, 1)))
        cf = QuadraticCostFunction(4, 1)
        cf.Q, cf.R = Q, R
        return synthesize_lqr_controller(ss, cf).K
    from scipy.linalg import solve_continuous_are
    S = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ S)


class LQR:
    nom = "LQR"
    commande_en_position = False

    def __init__(self, K, Ts=0.002, fc=50.0):
        self.K = np.atleast_2d(K)
        self.Ts = Ts
        self.d_al, self.d_th = DeriveeFiltree(fc, Ts), DeriveeFiltree(fc, Ts)

    def __call__(self, mes, t, ref_theta=0.0):
        x = np.array([mes["theta"] - ref_theta, mes["alpha"],
                      self.d_th(mes["theta"]), self.d_al(mes["alpha"])])
        return float(-(self.K @ x)[0])


# =========================================================================================
# Simulateur
# =========================================================================================
def simuler(ctrl, T=6.0, x0=(0, 3 * DEG, 0, 0), Ts=None, dt=0.00025, p=PARAM_EDUKIT,
            ref_theta=None, poussee=None, bruit=0.0, quantif=True, retard=0.0,
            calage=0.0, u_max=U_MAX, v_max=V_MAX, graine=0, garde=ALPHA_CHUTE):
    """
    Simule la boucle fermée. Retourne un dict de tableaux (t, theta, alpha, u, ...).
    ref_theta : fonction t -> consigne rotor (rad) ; poussee : (t0, dalpha) impulsion de vitesse
    calage    : erreur de verticale (rad), ex. socle incliné ; retard : retard pur (s)
    Pour PIDShawn, la sortie est un incrément de position exécuté par un profil moteur.
    """
    rng = np.random.default_rng(graine)
    Ts = Ts or ctrl.Ts
    n_sub = max(1, int(round(Ts / dt)))
    dt = Ts / n_sub
    N = int(T / Ts)
    x = np.array(x0, float)
    th_cible = x[0]                       # cible de position du moteur (PIDShawn)
    a_stepper = 39.3                      # 10 000 pas/s² en 1/8 de pas (sketch Arduino)
    v_stepper = 19.6                      # 5 000 pas/s en 1/8 de pas
    n_ret = int(round(retard / Ts))
    file_u = [0.0] * (n_ret + 1)
    log = {k: np.zeros(N) for k in ("t", "theta", "alpha", "u", "dtheta", "ref")}
    tombe = False
    for k in range(N):
        t = k * Ts
        r = ref_theta(t) if ref_theta else 0.0
        if poussee and abs(t - poussee[0]) < Ts / 2:
            x[3] += poussee[1]
        # --- mesure ---
        al_m, th_m = x[1] + calage, x[0]
        if quantif:
            res = getattr(ctrl, "resolution_codeur", RESOLUTION_CODEUR)
            al_m = np.round(al_m / res) * res
            th_m = np.round(th_m / (2 * np.pi / 3200)) * (2 * np.pi / 3200)
        al_m += bruit * rng.standard_normal()
        sortie = ctrl({"alpha": al_m, "theta": th_m}, t, r)
        file_u.append(sortie)
        cmd = file_u.pop(0)
        # --- actionneur + intégration ---
        if ctrl.commande_en_position:
            th_cible += cmd
        for _ in range(n_sub):
            if ctrl.commande_en_position:      # profil du driver : rejoindre la cible au plus vite
                err = th_cible - x[0]           # (accélération et vitesse bornées = trapèze)
                v_des = np.sign(err) * min(v_stepper, np.sqrt(2 * a_stepper * abs(err)))
                u = np.clip(400.0 * (v_des - x[2]), -a_stepper, a_stepper)
            else:
                u = np.clip(cmd, -u_max, u_max)
            if (x[2] >= v_max and u > 0) or (x[2] <= -v_max and u < 0):
                u = 0.0
            x = rk4(x, u, dt, p)
        log["t"][k], log["theta"][k], log["alpha"][k] = t, x[0], x[1]
        log["u"][k], log["dtheta"][k], log["ref"][k] = u, x[2], r
        if abs(x[1]) > garde or abs(x[0]) > THETA_MAX:
            tombe = True
            for key in log:
                log[key] = log[key][:k + 1]
            log["raison"] = "pendule tombé" if abs(x[1]) > garde else "butée rotor ±240°"
            break
    log["tombe"] = tombe
    log.setdefault("raison", "")
    return log


def metriques(log, t0=0.0, seuil_deg=0.5):
    """Indicateurs : temps de stabilisation de α (bande ±seuil), dépassement, effort, excursion."""
    t, al, th, u = log["t"], log["alpha"] / DEG, log["theta"] / DEG, log["u"]
    m = t >= t0
    hors = np.where(np.abs(al[m]) > seuil_deg)[0]
    ts = (t[m][hors[-1]] - t0) if len(hors) else 0.0
    Ts = t[1] - t[0] if len(t) > 1 else 1
    return dict(
        tombe=log["tombe"],
        t_stab=np.nan if log["tombe"] else ts,
        alpha_max=np.max(np.abs(al[m])) if m.any() else np.nan,
        theta_max=np.max(np.abs(th[m])) if m.any() else np.nan,
        u_max=np.max(np.abs(u[m])) if m.any() else np.nan,
        energie=np.sum(u[m] ** 2) * Ts,
        rms_alpha=np.sqrt(np.mean(al[m] ** 2)) if m.any() else np.nan,
    )


# =========================================================================================
# 4. (plus loin) LQI : LQR + intégrale de l'erreur rotor -> annule l'erreur statique
# =========================================================================================
def gain_lqi(Q, R, q_int, p=PARAM_EDUKIT):
    """État augmenté z = [θ, α, θ', α', ∫(θ-θref)] ; même Riccati (pyro) sur le modèle 5x5."""
    from pyro.dynamic.statespace import StateSpaceSystem
    from pyro.analysis.costfunction import QuadraticCostFunction
    from pyro.control.lqr import synthesize_lqr_controller
    A, B = matrices_lineaires(p)
    Aa = np.zeros((5, 5))
    Aa[:4, :4] = A
    Aa[4, 0] = 1.0
    Ba = np.vstack([B, [[0.0]]])
    Qa = np.zeros((5, 5))
    Qa[:4, :4] = Q
    Qa[4, 4] = q_int
    ss = StateSpaceSystem(Aa, Ba, np.eye(5), np.zeros((5, 1)))
    cf = QuadraticCostFunction(5, 1)
    cf.Q, cf.R = Qa, np.atleast_2d(R)
    return synthesize_lqr_controller(ss, cf).K


class LQI(LQR):
    nom = "LQI (LQR + intégrale)"

    def __init__(self, K, Ts=0.002, fc=50.0, z_max=2.0):
        LQR.__init__(self, K, Ts, fc)
        self.z, self.z_max = 0.0, z_max

    def __call__(self, mes, t, ref_theta=0.0):
        e = mes["theta"] - ref_theta
        self.z = np.clip(self.z + e * self.Ts, -self.z_max, self.z_max)
        x = np.array([e, mes["alpha"], self.d_th(mes["theta"]), self.d_al(mes["alpha"]), self.z])
        return float(-(self.K @ x)[0])
