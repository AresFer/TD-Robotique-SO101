# -*- coding: utf-8 -*-
"""
Jumeau numérique du SO-101 + « bancs d'essai » pour exécuter NOS SCRIPTS ORIGINAUX sans écran
et sans bras, afin de régénérer toutes les figures du compte rendu.

- HorlogeVirtuelle : remplace le module `time` d'un script ; time.sleep(dt) fait avancer une
  horloge virtuelle -> exécution déterministe et plus rapide que le temps réel.
- VisionneuseHorsEcran : remplace mujoco.viewer.launch_passive ; rend des images avec
  mujoco.Renderer (EGL) et produit un GIF ; recopie les marqueurs (user_scn) du script.
- BrasFeetechSimule : remplace lerobot_kinematics.feetech_arm. C'est un « jumeau » physique :
  les consignes de position sont envoyées aux actionneurs de position MuJoCo (kp = 17.8 N·m/rad,
  couple max 3.35 N·m, frottements et inerties des STS3215 du MJCF) et la gravité agit.
  feedback() renvoie donc une mesure réaliste : retard de suivi, affaissement sous la gravité.
"""
import os
import types

import numpy as np
import mujoco

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
# Pose de repos mesurée au début de l'essai réel (figures/reel/pick_place_astar_lqr_reel.jpg)
POSE_REPOS_REELLE = np.array([-0.24, -1.75, 1.65, 1.20, -1.15, 0.0])


class HorlogeVirtuelle(types.SimpleNamespace):
    def __init__(self, t0=1_000_000.0):
        super().__init__()
        self.t = t0
        self.time = lambda: self.t
        self.perf_counter = lambda: self.t
        self.monotonic = lambda: self.t

    def sleep(self, dt):
        self.t += max(0.0, float(dt))


class _Viewer:
    def __init__(self, model, data, largeur=480, hauteur=360, tous_les=3, camera=None, actif=True):
        self.m, self.d = model, data
        self.user_scn = mujoco.MjvScene(model, maxgeom=20000)
        self.frames = []
        self.k = 0
        self.tous_les = tous_les
        self.actif = actif
        self.r = mujoco.Renderer(model, hauteur, largeur) if actif else None
        self.cam = mujoco.MjvCamera()
        cam = camera or dict(azimuth=150, elevation=-28, distance=0.95, lookat=(0.18, 0.0, 0.08))
        self.cam.azimuth, self.cam.elevation = cam["azimuth"], cam["elevation"]
        self.cam.distance, self.cam.lookat[:] = cam["distance"], cam["lookat"]

    def is_running(self):
        return True

    def sync(self):
        self.k += 1
        if self.actif and self.k % self.tous_les == 0:
            self.frames.append(self.rendu())

    def rendu(self):
        self.r.update_scene(self.d, self.cam)
        sc = self.r.scene
        for i in range(self.user_scn.ngeom):
            if sc.ngeom >= sc.maxgeom:
                break
            g = self.user_scn.geoms[i]
            mujoco.mjv_initGeom(sc.geoms[sc.ngeom], g.type, g.size, g.pos, g.mat.flatten(), g.rgba)
            sc.ngeom += 1
        return self.r.render().copy()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


class VisionneuseHorsEcran:
    """Fabrique compatible avec mujoco.viewer.launch_passive(model, data)."""

    def __init__(self, **kw):
        self.kw = kw
        self.derniere = None

    def __call__(self, model, data, **_):
        self.derniere = _Viewer(model, data, **self.kw)
        return self.derniere

    def gif(self, chemin, fps=15, max_images=400):
        from PIL import Image
        fr = self.derniere.frames if self.derniere else []
        if not fr:
            return None
        pas = max(1, int(np.ceil(len(fr) / max_images)))
        imgs = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=128) for f in fr[::pas]]
        os.makedirs(os.path.dirname(chemin), exist_ok=True)
        imgs[0].save(chemin, save_all=True, append_images=imgs[1:], duration=int(1000 * pas / fps), loop=0,
                     optimize=True)
        return chemin


class _Materiel:
    def __init__(self, bras):
        self.bras = bras

    def write(self, nom, valeur):
        if nom == "Torque_Enable":
            self.bras.couple = bool(np.atleast_1d(valeur)[0])
        # Goal_Position en degrés avec signes Feetech : ignoré (le jumeau démarre déjà sur q0)

    def read(self, nom):
        return np.rad2deg(self.bras.feedback())

    def set_calibration(self, *_):
        pass


class BrasFeetechSimule:
    """Jumeau physique du bras réel (même interface que lerobot_kinematics.feetech_arm).

    Co-simulation avec l'horloge virtuelle du script : la physique MuJoCo avance exactement du
    temps écoulé sur cette horloge. t_ecriture / t_lecture simulent la durée des transactions
    sur le bus série Feetech (écriture Goal_Position, lecture Present_Position des 6 servos) :
    si elles dépassent DT, la boucle du script ralentit, comme sur le vrai PC."""

    def __init__(self, xml, q0=POSE_REPOS_REELLE, dt_boucle=0.02, horloge=None, t_ecriture=0.0,
                 t_lecture=0.0, kp=None, bruit=0.0, graine=0):
        self.m = mujoco.MjModel.from_xml_path(xml) if isinstance(xml, str) else xml
        self.d = mujoco.MjData(self.m)
        self.qadr = np.array([self.m.jnt_qposadr[self.m.joint(n).id] for n in JOINTS])
        self.act = np.array([self.m.actuator(n).id for n in JOINTS])
        if kp is not None:                       # modification du gain des servos (étude)
            for a in self.act:
                self.m.actuator_gainprm[a, 0] = kp
                self.m.actuator_biasprm[a, 1] = -kp
        self.d.qpos[self.qadr] = q0
        self.d.ctrl[self.act] = q0
        mujoco.mj_forward(self.m, self.d)
        self.dt_boucle = dt_boucle
        self.horloge = horloge
        self.t_ecriture, self.t_lecture = t_ecriture, t_lecture
        self.t_phys = horloge.t if horloge else 0.0
        self.couple = True
        self.bruit = bruit
        self.rng = np.random.default_rng(graine)
        self.arm_hardware = _Materiel(self)
        self.historique = []          # (t, consigne, mesure) à chaque écriture
        self.periodes = []            # durée réelle de chaque tour de boucle
        self._t_derniere = None

    def _avancer(self, dt=None):
        if self.horloge is None:
            dt = self.dt_boucle if dt is None else dt
            cible = self.t_phys + dt
        else:
            cible = self.horloge.t
        n = int(round((cible - self.t_phys) / self.m.opt.timestep))
        for _ in range(max(0, n)):
            mujoco.mj_step(self.m, self.d)
        self.t_phys += max(0, n) * self.m.opt.timestep

    def action(self, qpos_rad):
        if self.horloge is not None:
            if self._t_derniere is None:          # début de la boucle : la physique part d'ici
                self.t_phys = self.horloge.t
            self._avancer()
            if self._t_derniere is not None:
                self.periodes.append(self.horloge.t - self._t_derniere)
            self._t_derniere = self.horloge.t
        cible = np.array(qpos_rad, float)
        self.d.ctrl[self.act] = np.clip(cible, self.m.actuator_ctrlrange[self.act, 0],
                                        self.m.actuator_ctrlrange[self.act, 1])
        if self.horloge is not None:
            self.horloge.sleep(self.t_ecriture)
            self.horloge.sleep(self.t_lecture)     # action() relit aussi Present_Position
            self._avancer()
        else:
            self._avancer(self.dt_boucle)
        self.historique.append((self.t_phys, cible.copy(), self.d.qpos[self.qadr].copy()))
        return self._mesure()

    def feedback(self):
        # la lecture initiale (avant la boucle de commande) est supposée instantanée
        if self.horloge is not None and self._t_derniere is not None:
            self.horloge.sleep(self.t_lecture)
            self._avancer()
        return self._mesure()

    def _mesure(self):
        q = self.d.qpos[self.qadr].copy()
        if self.bruit:
            q += self.bruit * self.rng.standard_normal(6)
        return q

    def disconnect(self):
        pass
