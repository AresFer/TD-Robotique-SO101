- Script original + lerobot_IK d'origine : **échec** — `IK : cible [0.22 0.   0.06] non atteinte (erreur 32.4 mm). Point hors de portée ? Change x/z.`
- Correctif, jumeau : durée 14.6 s ; erreur mesurée max par articulation [0.175, 0.13, 0.139, 0.121, 0.0] rad ; erreur statique d'épaule (gravité) ≈ +0.030 rad

| Q position | Q vitesse | R | kp = √(Q/R) | kd | ωn (rad/s) | ζ | temps de réponse 5 % |
|---|---|---|---|---|---|---|---|
| 1 | 1 | 1 | 1.00 | 1.732 | 1.00 | 0.866 | 3.00 s |
| 9 | 1 | 1 | 3.00 | 2.646 | 1.73 | 0.764 | 1.83 s |
| 81 | 1 | 1 | 9.00 | 4.359 | 3.00 | 0.726 | 0.99 s |
| 729 | 1 | 1 | 27.00 | 7.416 | 5.20 | 0.714 | 0.56 s |
| 81 | 30 | 1 | 9.00 | 6.928 | 3.00 | 1.155 | 1.95 s |

| Q position | durée (s) | max\|x − consigne\| (rad) | max\|mesure − x\| (rad) | max\|u\| (rad/s²) |
|---|---|---|---|---|
| 1 | 33.6 | 0.093 | 0.173 | 24.6 |
| 9 | 20.2 | 0.081 | 0.174 | 24.7 |
| 81 | 14.6 | 0.085 | 0.175 | 24.9 |
| 729 | 12.3 | 0.088 | 0.175 | 25.7 |
