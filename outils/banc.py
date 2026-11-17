# -*- coding: utf-8 -*-
"""
Banc d'essai : charge un script ORIGINAL de partie2_so101/ (sans le modifier) dans un espace de
travail qui imite la racine de lerobot-kinematics, puis remplace uniquement les entrées/sorties :
écran MuJoCo, horloge, bras réel (-> jumeau numérique), plt.show (-> fichiers PNG).

Pré-requis : un clone de https://github.com/box2ai-robotics/lerobot-kinematics (pour la scène
scene_so101.xml et ses maillages). Chemin pris dans la variable LEROBOT_KINEMATICS_DIR,
sinon ../lerobot-kinematics à côté de ce dépôt.
"""
import importlib.util
import os
import sys

import numpy as np

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RACINE, "outils", "lerobot_shim"))
sys.path.insert(0, RACINE)

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402

from outils.jumeau import HorlogeVirtuelle, VisionneuseHorsEcran, BrasFeetechSimule, POSE_REPOS_REELLE  # noqa: E402

LK = os.environ.get("LEROBOT_KINEMATICS_DIR", os.path.join(os.path.dirname(RACINE), "lerobot-kinematics"))
WS = os.path.join(RACINE, ".banc", "ws")
EX = os.path.join(WS, "examples")


def espace_travail():
    """Crée .banc/ws/examples avec la scène SO-101 (liens) et nos scripts (liens)."""
    src = os.path.join(LK, "examples")
    if not os.path.exists(os.path.join(src, "scene_so101.xml")):
        raise SystemExit(f"Scène introuvable : clonez lerobot-kinematics dans {LK} "
                         "ou définissez LEROBOT_KINEMATICS_DIR")
    os.makedirs(EX, exist_ok=True)
    for f in ("scene_so101.xml", "so101_new_calib.xml", "assets_so101"):
        dst = os.path.join(EX, f)
        if not os.path.exists(dst):
            os.symlink(os.path.join(src, f), dst)
    for f in os.listdir(os.path.join(RACINE, "partie2_so101")):
        if f.endswith(".py"):
            dst = os.path.join(EX, f)
            if os.path.lexists(dst):
                os.remove(dst)
            os.symlink(os.path.join(RACINE, "partie2_so101", f), dst)
    return EX


def charger(nom):
    """Importe examples/<nom>.py (exécute son en-tête : os.chdir vers la racine simulée)."""
    espace_travail()
    chemin = os.path.join(EX, nom + ".py")
    spec = importlib.util.spec_from_file_location(nom + "_banc", chemin)
    mod = importlib.util.module_from_spec(spec)
    sys.path.insert(0, EX)
    spec.loader.exec_module(mod)
    return mod


class Capture:
    """Remplace plt.show : enregistre les figures ouvertes sous un préfixe donné."""

    def __init__(self, dossier, prefixe):
        self.dossier, self.prefixe, self.n = dossier, prefixe, 0
        self.fichiers = []

    def __call__(self, *a, **k):
        if not self.dossier:
            plt.close("all")
            return
        os.makedirs(self.dossier, exist_ok=True)
        for num in plt.get_fignums():
            fig = plt.figure(num)
            self.n += 1
            ch = os.path.join(self.dossier, f"{self.prefixe}_{self.n}.png")
            fig.savefig(ch, dpi=120, bbox_inches="tight")
            self.fichiers.append(ch)
            print("  figure ->", os.path.relpath(ch, RACINE))
        plt.close("all")


def ecrire_points(qA, qB):
    """Écrit examples/points_AB.json (équivalent du MODE « teach ») dans l'espace de travail."""
    import json
    espace_travail()
    with open(os.path.join(EX, "points_AB.json"), "w") as f:
        json.dump({"A": list(map(float, qA)), "B": list(map(float, qB))}, f, indent=2)


def effacer_points():
    ch = os.path.join(EX, "points_AB.json")
    if os.path.exists(ch):
        os.remove(ch)


# Points A/B lus sur le graphe de l'essai réel (figures/reel/pick_place_astar_lqr_reel.jpg), ±0.05 rad
QA_REEL = [0.75, 0.88, 0.22, -1.08, -1.15, 1.0]
QB_REEL = [-0.65, 0.90, 0.45, -1.12, -1.15, 1.0]


def executer(nom_script, mode, prefixe, dossier_fig, gif=None, q0_jumeau=POSE_REPOS_REELLE,
             reglages=None, kw_jumeau=None, kw_vue=None):
    """Exécute main() d'un script original. mode = 'sim' ou 'real' (-> jumeau numérique).
    reglages : dict d'attributs du module à modifier AVANT main() (études paramétriques).
    Retourne (module, logs) ; logs = dict(t, q, ref, u, meas) intercepté dans plot_logs."""
    mod = charger(nom_script)
    mod.MODE = mode
    for k, v in (reglages or {}).items():
        setattr(mod, k, v)
    horloge = HorlogeVirtuelle()
    mod.time = horloge
    vue = VisionneuseHorsEcran(**(kw_vue or {}))
    mujoco.viewer.launch_passive = vue
    cap = Capture(dossier_fig, prefixe)
    plt.show = cap
    import lerobot_kinematics as lk
    jumeaux = []

    def fabrique(driver_port=None, calibration_file=None):
        b = BrasFeetechSimule(os.path.join(EX, "scene_so101.xml"), q0=q0_jumeau, dt_boucle=mod.DT,
                              horloge=horloge, **(kw_jumeau or {}))
        jumeaux.append(b)
        return b
    lk.feetech_arm = fabrique
    logs = {}
    orig = mod.plot_logs

    def plot_logs(t, q, ref, u, meas):
        logs.update(t=np.array(t), q=np.array(q), ref=np.array(ref), u=np.array(u), meas=np.array(meas))
        if dossier_fig:
            orig(t, q, ref, u, meas)
    if orig.__code__.co_argcount == 5:          # signature plot_logs(t, q, ref, u, meas)
        mod.plot_logs = plot_logs
    if not dossier_fig:
        mod.pyro_step_response = lambda *a, **k: None
    with matplotlib.style.context("default"):     # figures des scripts : style matplotlib d'origine
        logs["retour"] = mod.main()
    if gif:
        vue.gif(gif)
        print("  gif    ->", os.path.relpath(gif, RACINE))
    logs["jumeau"] = jumeaux[0] if jumeaux else None
    logs["vue"] = vue
    return mod, logs
