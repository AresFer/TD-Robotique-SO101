# -*- coding: utf-8 -*-
"""
Modèle du pendule inversé rotatif EDUKIT (STEVAL-EDUKIT01) — Partie 1 du TD (Q1 à Q3).

Notations
---------
    θ  (theta) : angle du rotor (bras horizontal entraîné par le moteur pas-à-pas)   [rad]
    α  (alpha) : angle du pendule mesuré DEPUIS LA VERTICALE HAUTE (α = 0 : équilibre)  [rad]
    u          : accélération du rotor commandée au moteur pas-à-pas, u = θ''           [rad/s²]

Le firmware EDUKIT pilote le moteur pas-à-pas en ACCÉLÉRATION (fonction apply_acceleration) :
le rotor obéit donc à θ'' = u, quelle que soit la charge (tant que le couple suffit).

Équation du pendule (Lagrange, tige + masse en bout, cf. guide EDUKIT / Tutorial 5) :

    α'' = ωg² sin α  +  ωr² cos α · u  +  θ'² sin α cos α  -  γ α'

    ωg² = m g lc / J   (gravité)      ωr² = m r lc / J  (couplage rotor -> pendule)
    γ   = c / J        (frottement visqueux du pivot)

Linéarisé autour de α = 0 (pendule en haut) :

    x = [θ, α, θ', α']ᵀ        x' = A x + B u

        | 0   0     1    0 |          | 0   |
    A = | 0   0     0    1 |      B = | 0   |
        | 0   0     0    0 |          | 1   |
        | 0  ωg²    0   -γ |          | ωr² |

Valeurs numériques (Tutorial 5 du guide EDUKIT, configuration pendule en haut) :
    ωg² = 55.85 rad²/s²    ωr² = 0.803 rad/s² par rad/s²  (=> r ≈ ωr²·g/ωg² ≈ 0.141 m)
    facteur de qualité Q ≈ 20  => γ = ωg / Q ≈ 0.374 s⁻¹
    Tige de 0.26 m, bras rotor r ≈ 0.14 m, période d'échantillonnage du firmware 2 ms.
Le système est donc instable : pôle réel +sqrt(ωg²) ≈ +7.5 rad/s (temps de doublement ≈ 93 ms).
"""
import numpy as np

try:
    from pyro.dynamic import system
    from pyro.dynamic.statespace import StateSpaceSystem, linearize
    PYRO = True
except ImportError:          # le modèle reste utilisable sans pyro
    PYRO = False

# -----------------------------------------------------------------------------------------
# Paramètres physiques (Tutorial 5 EDUKIT)
# -----------------------------------------------------------------------------------------
PARAM_EDUKIT = dict(
    wg2=55.85,       # ωg²  [rad²/s²]
    wr2=0.803,       # ωr²  [-]
    gamma=np.sqrt(55.85) / 20.0,   # γ = ωg/Q avec Q = 20  [1/s]
)

# Limites de l'actionneur (moteur pas-à-pas + driver L6474, firmware EDUKIT)
U_MAX = 60.0         # accélération rotor max (rad/s²)  ~ 9 700 pas/s² en 1/16 de pas
V_MAX = 20.0         # vitesse rotor max (rad/s)        ~ 10 000 pas/s (MAX_SPEED_UPPER_INIT)
THETA_MAX = np.deg2rad(240)   # butées logicielles du rotor (ROTOR_POSITION_*_LIMIT)
ALPHA_CHUTE = np.deg2rad(20)  # au-delà : pendule considéré tombé (garde ±20° du code Shawn Hymel)

# Capteur : codeur optique 600 impulsions/tour -> 2400 fronts/tour en quadrature
RESOLUTION_CODEUR = 2 * np.pi / 2400       # 0.15°


def matrices_lineaires(p=PARAM_EDUKIT):
    """Matrices (A, B) du modèle linéarisé autour du pendule en haut."""
    A = np.array([[0, 0, 1, 0],
                  [0, 0, 0, 1],
                  [0, 0, 0, 0],
                  [0, p["wg2"], 0, -p["gamma"]]], float)
    B = np.array([[0], [0], [1], [p["wr2"]]], float)
    return A, B


def f_nonlineaire(x, u, p=PARAM_EDUKIT):
    """Dynamique non linéaire complète x' = f(x, u)."""
    th, al, dth, dal = x
    ddth = u
    ddal = (p["wg2"] * np.sin(al) + p["wr2"] * np.cos(al) * u
            + dth ** 2 * np.sin(al) * np.cos(al) - p["gamma"] * dal)
    return np.array([dth, dal, ddth, ddal])


def rk4(x, u, dt, p=PARAM_EDUKIT):
    k1 = f_nonlineaire(x, u, p)
    k2 = f_nonlineaire(x + 0.5 * dt * k1, u, p)
    k3 = f_nonlineaire(x + 0.5 * dt * k2, u, p)
    k4 = f_nonlineaire(x + dt * k3, u, p)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


# -----------------------------------------------------------------------------------------
# Version pyro du modèle (ContinuousDynamicSystem) : permet linearize(), ctl + sys, etc.
# -----------------------------------------------------------------------------------------
if PYRO:
    class PenduleRotatifEdukit(system.ContinuousDynamicSystem):
        """Pendule inversé rotatif EDUKIT au format pyro (n=4 états, m=1 entrée)."""

        def __init__(self, p=PARAM_EDUKIT):
            system.ContinuousDynamicSystem.__init__(self, 4, 1, 4)
            self.params = dict(p)
            self.name = "Pendule rotatif EDUKIT"
            self.state_label = ["θ rotor", "α pendule", "dθ", "dα"]
            self.state_units = ["[rad]", "[rad]", "[rad/s]", "[rad/s]"]
            self.input_label = ["u = θ''"]
            self.input_units = ["[rad/s²]"]
            self.output_label = self.state_label
            self.u_ub = np.array([U_MAX])
            self.u_lb = np.array([-U_MAX])
            self.x_ub = np.array([THETA_MAX, np.pi, V_MAX, 30])
            self.x_lb = -self.x_ub
            # graphique 3D : rotor de 0.14 m, pendule de 0.26 m
            self.is_3d = True
            self.domain = [(-0.35, 0.35), (-0.35, 0.35), (-0.05, 0.4)]

        def f(self, x, u, t=0):
            return f_nonlineaire(x, float(np.atleast_1d(u)[0]), self.params)

        def forward_kinematic_domain(self, q):
            return self.domain

        def forward_kinematic_lines(self, q):
            th, al = q[0], q[1]
            r, L = 0.14, 0.26
            pivot = np.array([r * np.cos(th), r * np.sin(th), 0.2])
            tangente = np.array([-np.sin(th), np.cos(th), 0.0])
            bout = pivot + L * (np.sin(al) * tangente + np.array([0, 0, np.cos(al)]))
            rotor = np.array([[0, 0, 0.2], pivot])
            pendule = np.array([pivot, bout])
            socle = np.array([[0, 0, 0], [0, 0, 0.2]])
            return [socle, rotor, pendule], ["k-", "b-", "r-"], [None, None, None]

    def modele_pyro_lineaire(p=PARAM_EDUKIT):
        """Linéarisation NUMÉRIQUE par pyro (différences finies) du modèle non linéaire."""
        sysnl = PenduleRotatifEdukit(p)
        sysnl.xbar = np.zeros(4)
        sysnl.ubar = np.zeros(1)
        return linearize(sysnl, 1e-4)


if __name__ == "__main__":
    A, B = matrices_lineaires()
    print("A =\n", A, "\nB =\n", B.T)
    print("valeurs propres de A :", np.round(np.linalg.eigvals(A), 3))
    if PYRO:
        ss = modele_pyro_lineaire()
        print("écart A analytique / A pyro :", np.abs(ss.A - A).max())
        print("écart B analytique / B pyro :", np.abs(ss.B - B).max())
