# Partie 2 — Scripts pour le bras LeRobot SO-101

Ces scripts se lancent **depuis la racine du dépôt lerobot-kinematics**, après les avoir copiés
dans son dossier `examples/` (ils utilisent `examples/scene_so101.xml` et
`examples/main_follower.json`). Chaque script a une variable `MODE` en tête :
`"sim"` (MuJoCo seul), `"real"` (MuJoCo + bras), `"teach"` (bras libre, on enregistre des points).

| Fichier | Statut | Question | Rôle |
|---|---|---|---|
| `calibration_bras2.py` | original | Q4 | centre chaque servo à 2048 sur la pose de référence, vérifie les sens, écrit `main_follower.json` |
| `pid_ik_sim.py` | original | Q5 | PID articulaire piloté par l'IK, modèle J/b simulé |
| `pid_ik_real.py` | original | Q5 | même PID comme générateur de consignes pour le vrai bras (écrit pour le modèle SO-100) |
| `pick_place_trajectoire_LQR.py` | original | Q6-Q7 | pick & place, trajectoires cartésiennes + LQR pyro |
| `pick_place_astar.py` | original | Q8 | pick & place avec planification A* et obstacles + LQR pyro |
| `astar_planner.py` | **reconstruit** | Q8 | planificateur A* 3D importé par `pick_place_astar.py` (l'original n'a pas été conservé) |
| `lqr_etoile.py` | nouveau | Q6 | A → B en ligne droite puis parcours en étoile (pentagramme ou rayons) |
| `ik_precise.py` | nouveau | Q5 | correctif de `lerobot_IK` (tolérance) : `from ik_precise import lerobot_IK_precis as lerobot_IK` |
| `rrt_planner.py` | nouveau | bonus | RRT / RRT* dans l'espace articulaire, même modèle de collision qu'A* |
| `dmp.py` | nouveau | bonus | Dynamic Movement Primitives : apprendre un geste (`teach`) et le rejouer |

Les fichiers « originaux » sont exactement ceux utilisés pendant les TD (non modifiés) ; les
défauts trouvés et leurs correctifs sont décrits dans [rapport/10](../rapport/10_ameliorations.md).
