# Études de simulation (Partie 2)

Chaque script exécute un ou plusieurs scripts de [`partie2_so101/`](../partie2_so101) **sans les
modifier** (banc d'essai [`outils/banc.py`](../outils/banc.py) : jumeau numérique, horloge
virtuelle, rendu hors écran) et écrit ses figures dans `figures/` et ses chiffres dans un `.md`.

| Script | Question | Contenu |
|---|---|---|
| `s1_cinematique.py` | Q4-Q5 | cohérence FK lerobot / MuJoCo, comportement de `lerobot_IK`, espace de travail |
| `s2_pid_ik.py` | Q5 | `pid_ik_sim.py` original et corrigé, pôles et balayages du PID |
| `s3_bras_pyro_2d_3d.py` | Q5 | bras 2 DDL (PID / LQR linéarisé / couple calculé + LQR), 5 DDL, 3D, étoile 2D |
| `s4_lqr_pick_place.py` | Q6-Q7 | `pick_place_trajectoire_LQR.py` : tel quel, corrigé, jumeau, balayage de Q, théorie du LQR |
| `s5_etoile.py` | Q6 | `lqr_etoile.py` : pentagramme et rayons sur le jumeau, influence de Q et de VEL_MAX |
| `s6_astar_obstacles.py` | Q8 | `pick_place_astar.py` : jumeau, scénario réel reconstruit, période de boucle, A* vs Dijkstra, zones interdites |
| `s7_rrt.py` | bonus | RRT / RRT* contre A* |
| `s8_dmp.py` | bonus | DMP : généralisation, évitement, exécution sur le jumeau |
| `s9_ameliorations.py` | bonus | pics de commande (spline + durée recalée) et boucle lente (dt mesuré) |

Pré-requis : voir [rapport/00 § 4](../rapport/00_methodologie.md#4-reproduire-toutes-les-figures).
