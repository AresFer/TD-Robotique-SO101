| Variante | durée | max\|u\| (rad/s²) | ∫u² dt | max\|état − consigne\| (rad) | écart pince au chemin planifié (mm, moy / max) | configurations de l'état en collision | max\|mesure − état\| jumeau (rad) |
|---|---|---|---|---|---|---|---|
| interpolation linéaire + différences finies (scripts actuels) | 2.3 s | 15.0 | 412 | 0.811 | 42.5 / 106.1 | 0 / 117 | 0.220 |
| spline + durée recalée (correctif) : T = 13.83 s au lieu de 1.33 s | 14.8 s | 10.0 | 29 | 0.009 | 0.8 / 3.2 | 0 / 742 | 0.128 |

| Période de boucle | retard max (DT fixe) | retard max (dt mesuré) |
|---|---|---|
| 20 ms | 0.009 rad | 0.009 rad |
| 25 ms | 0.035 rad | 0.011 rad |
| 30 ms | 0.074 rad | 0.015 rad |
| 35 ms | 0.109 rad | 0.017 rad |
| 40 ms | 0.143 rad | 0.016 rad |
| 50 ms | 0.213 rad | 0.024 rad |
