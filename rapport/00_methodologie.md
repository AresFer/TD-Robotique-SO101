# Méthodologie : réel, simulation, jumeau numérique — et comment tout reproduire

> [Sommaire](../README.md) · [Q1-Q2 : pendule PID →](01_pendule_PID.md)

## 1. Ce qui est réel et ce qui est simulé

Nous distinguons systématiquement trois niveaux de preuve, indiqués en tête de chaque chapitre :

| Niveau | Signification | Où |
|---|---|---|
| **Réel** | mesuré sur le vrai SO-101 | [`figures/reel/`](../figures/reel) : graphe du pick & place A* + LQR et capture MuJoCo de l'essai ; photos/vidéos dans [`media/`](../media/README.md) |
| **Simulation** | modèle (MuJoCo, pyro, modèle EDUKIT) commandé par nos scripts | toutes les autres figures |
| **Jumeau numérique** | nos scripts **originaux, non modifiés**, exécutés en `MODE = "real"`, mais le bras est remplacé par un bras MuJoCo *physique* (gravité, servos de position) | figures marquées « jumeau » |

Aucune figure de simulation n'est présentée comme une mesure réelle. Pour la partie 1 (pendule
EDUKIT), nous n'avons pas de mesures : tout est simulé, avec un protocole d'essai proposé.

## 2. Le jumeau numérique du SO-101

[`outils/jumeau.py`](../outils/jumeau.py) et [`outils/banc.py`](../outils/banc.py) permettent
d'exécuter les scripts de [`partie2_so101/`](../partie2_so101) **sans les modifier** :

| Élément remplacé | Par | Pourquoi |
|---|---|---|
| `feetech_arm` (bras réel) | `BrasFeetechSimule` : consignes → actionneurs de position MuJoCo (kp = 17.8 N·m/rad, couple max 3.35 N·m, frottements et inerties des STS3215 du fichier MJCF), gravité | obtenir une « mesure » réaliste : retard de suivi, affaissement |
| module `time` du script | `HorlogeVirtuelle` : `sleep(dt)` avance une horloge | exécution déterministe, plus rapide que le temps réel |
| `mujoco.viewer.launch_passive` | `VisionneuseHorsEcran` : rendu `mujoco.Renderer` (EGL) → GIF | pas d'écran sur le serveur |
| `plt.show()` | enregistrement PNG | figures versionnées |
| temps de communication du bus | `t_ecriture`, `t_lecture` dans le jumeau (co-simulation avec l'horloge) | tester l'hypothèse « boucle réelle plus lente que DT » ([Q8](07_evitement_obstacles.md#32-scénario-réel-reconstruit-et-vitesse-réelle-de-la-boucle)) |

Limites assumées : les servos MuJoCo sont plus raides que les vrais (le jumeau sous-estime
l'erreur de suivi d'un facteur ~4 à vitesse égale) ; la saisie d'objet n'est pas simulée
physiquement.

## 3. Bibliothèques

| Bibliothèque | Usage | Remarque |
|---|---|---|
| [pyro](https://github.com/SherbyRobotics/pyro) | LQR (`synthesize_lqr_controller`), modèles `StateSpaceSystem`, `linearize`, manipulateurs 2D/3D, `JointPID`, contrôleurs d'effecteur | utilisée dans toutes les synthèses LQR du dépôt |
| [lerobot-kinematics](https://github.com/box2ai-robotics/lerobot-kinematics) | FK / IK du SO-101, pilote Feetech, scène MuJoCo `scene_so101.xml` | sans compilateur : [`outils/lerobot_shim`](../outils/lerobot_shim/lerobot_kinematics/__init__.py) (copie fidèle sur Robotics Toolbox) |
| [MuJoCo](https://mujoco.org) | visualisation, collisions (`mj_geomDistance`), jumeau physique | 3.x |
| NumPy / SciPy / Matplotlib | calcul, splines, figures | |

## 4. Reproduire toutes les figures

```bash
git clone https://github.com/box2ai-robotics/lerobot-kinematics.git ../lerobot-kinematics   # scène + maillages
pip install -r requirements.txt
export LEROBOT_KINEMATICS_DIR=../lerobot-kinematics
export MUJOCO_GL=egl          # rendu sans écran (Linux) ; inutile sous Windows avec écran

# Partie 1 (≈ 4 min)
python partie1_pendule/validation_modele.py
python partie1_pendule/q1_q2_pid.py
python partie1_pendule/q3_lqr.py
python partie1_pendule/robustesse_pendule.py
# Partie 2 (≈ 20 min)
python simulations/s1_cinematique.py
python simulations/s2_pid_ik.py
python simulations/s3_bras_pyro_2d_3d.py
python simulations/s4_lqr_pick_place.py
python simulations/s5_etoile.py
python simulations/s6_astar_obstacles.py
python simulations/s7_rrt.py
python simulations/s8_dmp.py
python simulations/s9_ameliorations.py
```

Chaque script écrit ses figures dans `figures/` et ses chiffres dans un fichier `.md` à côté
(les tableaux du rapport en sont extraits). Les durées de planification A* varient de ±10 %
selon la machine ; les durées des mouvements simulés peuvent varier de quelques pour cent d'une
exécution à l'autre (chemin A* légèrement différent).

## 5. Outils d'IA

Le sujet autorise les outils d'IA générative. Nous les avons utilisés pour structurer le dépôt,
écrire les scripts d'étude et le banc de simulation, et relire le code ; les choix, les essais
sur le vrai bras et la validation des résultats restent les nôtres.
