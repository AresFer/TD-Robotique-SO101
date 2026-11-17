# Bilan : ce que nous avons appris, et ce que nous ferions ensuite

> [← Améliorations](10_ameliorations.md) · [Sommaire](../README.md)

## 1. Les algorithmes du TD, côte à côte

| Algorithme | Où | Modèle utilisé | Ce qu'il fait bien | Sa limite, mesurée |
|---|---|---|---|---|
| **PID** mono-boucle (Shawn Hymel) | pendule | aucun | 3 gains, se règle à la main | ne régule pas le rotor : butée en ~5 s ; île de stabilité minuscule ; gains ∝ 1/Δt |
| **Double PID** (EDUKIT) | pendule | aucun | stabilise pendule **et** rotor, économe | marge de phase 27° ; erreur statique 7.5° si le socle penche de 0.5° |
| **PID** + IK (`pid_ik_sim`) | bras (modèle J, b) | par articulation | simple, sans dépassement si Kd ≈ 0.15–0.5 | intégrale inutile sans gravité : queue de ~4 s |
| **LQR** (pyro) | pendule | linéarisé 4 états | réglage par 2–3 poids physiques, marge de phase ≥ 60° garantie, retard toléré 60 ms | pas d'action intégrale (→ LQI) ; sature si R trop petit |
| **LQI** | pendule | augmenté 5 états | erreur statique 0.3° | un peu plus nerveux que le LQR |
| **LQR** double intégrateur | SO-101 | $\ddot q = u$ | consignes lisses, réglage $\omega_n = (Q/R)^{1/4}$ | ne voit pas la mesure : précision fixée par les servos (−7.5 mm de gravité) |
| **Couple calculé + LQR** | bras 2 DDL (pyro) | dynamique complète | même comportement partout | dépend d'un modèle exact |
| **A*** | SO-101 | MuJoCo (collisions) | chemin le plus court de la pince, déterministe, bras entier vérifié | 7 s de calcul ; le suivi LQR peut sortir du couloir (106 mm) |
| **Dijkstra** | SO-101 | idem | même optimum | ×10.7 cases développées |
| **RRT / RRT*** | SO-101 | idem | 15× plus rapide qu'A* | chemins 2–4 fois plus longs et aléatoires |
| **DMP** | SO-101 | aucun (apprentissage) | rejoue un geste vers un autre but / à une autre vitesse | ne garantit rien vis-à-vis des obstacles (109/196 configurations en collision) |

## 2. Les cinq résultats qui nous semblent les plus importants

1. **Le seuil de convergence de `lerobot_IK` vaut 45 mm, pas 1 mm** (tolérance appliquée à une
   erreur quadratique). Il explique pourquoi `pid_ik_sim.py` finit à 28 mm de sa cible et pourquoi
   le premier script LQR échoue. Correctif d'une ligne.
2. **La boucle réelle tournait vraisemblablement à ~31 Hz au lieu de 50 Hz** : c'est la seule
   hypothèse testée qui reproduit à la fois la durée (35.5 s) et le retard état/consigne du graphe
   réel. Vérifiable en 2 minutes sur le banc.
3. **Avec l'anticipation, Q change la durée, pas la précision** : sur le SO-101, au-delà de
   Q ≈ 81, la précision est limitée par les servos et la gravité, pas par le LQR.
4. **Un planificateur sans collision ne suffit pas** : si la loi horaire demande plus que ce que
   les articulations peuvent faire, l'état sort du chemin planifié (jusqu'à 10 cm en simulation).
5. **Sur le pendule, le LQR et le double PID ont la même structure** (retour d'état, mêmes
   signes) ; le LQR apporte une méthode de réglage et des marges garanties (60° contre 27°).

## 3. Limites de ce travail

* Partie 1 entièrement simulée (paramètres du guide EDUKIT, ωg² mesuré sur un autre kit : 48
  au lieu de 55.85) ; les PID réels peuvent se comporter différemment, en particulier le PID de
  Shawn Hymel dont la stabilité dépend de la période de la boucle série.
* `astar_planner.py` est une reconstruction : les statistiques de planification ne sont pas
  celles du planificateur utilisé pendant l'essai réel.
* Le jumeau numérique sous-estime l'erreur de suivi des vrais servos (facteur ~4).
* La saisie physique d'objets n'est pas simulée.

## 4. Prochaines expériences sur le banc (dans l'ordre)

1. Mesurer la période réelle de la boucle (`time.time()` dans la boucle, 1 ligne).
2. Remplacer `lerobot_IK` par `lerobot_IK_precis` et relancer `pid_ik_real.py` (adapté au SO-101).
3. Intégrer avec le dt mesuré + loi horaire recalée ; refaire le pick & place A* et comparer le
   panneau « erreur mesurée » au graphe réel actuel.
4. Identifier un servo (échelon 0.5 rad, 100 Hz) pour rendre le jumeau fidèle.
5. Filmer l'étoile (stylo dans la pince, vue de dessus) : erreur réelle contre 3.7 mm simulés.
6. Pendule EDUKIT : protocole du [chapitre 1](01_pendule_PID.md#protocole-proposé-pour-lessai-réel-à-compléter-avec-vos-mesures).
