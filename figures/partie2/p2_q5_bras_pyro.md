Cible [0.45, 0.35] -> IK q_d = [0.3564, 1.6208]

K (LQR linéarisé) =
[[17.73 -0.21  2.92  0.05]
 [-0.81  9.63 -0.05  0.7 ]]

| Contrôleur | erreur finale effecteur | temps pour entrer à ±0.02 rad | couple max |
|---|---|---|---|
| PID articulaire (pyro JointPID) | 39.16 mm | 4.00 s | 10.6 N·m |
| LQR sur modèle linéarisé | 0.00 mm | 0.62 s | 14.4 N·m |
| couple calculé + LQR | 0.03 mm | 2.08 s | 3.4 N·m |
| 5 DDL cinématique : erreur finale | 3.67 mm |  |  |
| 3 DDL 3D (PID effecteur) : erreur finale | 2.98 mm |  |  |
| Étoile 2 DDL : écart moyen / max à la consigne | 9.5 mm / 25.5 mm |  |  |
