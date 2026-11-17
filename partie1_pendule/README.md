# Partie 1 — Pendule inversé rotatif EDUKIT (Q1 à Q3)

| Fichier | Rôle |
|---|---|
| `modele_pendule.py` | équations du pendule EDUKIT (non linéaire + linéarisé), classe **pyro** `PenduleRotatifEdukit` |
| `commandes.py` | PID de Shawn Hymel (transcription fidèle), double PID du firmware EDUKIT, LQR et LQI (pyro), simulateur réaliste (codeur, retard, saturations, bruit) |
| `validation_modele.py` | Q3 : vérifier la dynamique (oscillation libre, chute, linéarisation pyro) |
| `q1_q2_pid.py` | Q1-Q2 : PID nominal, balayages Kp/Ki/Kd/Δt, cartes de stabilité |
| `q3_lqr.py` | Q3 : LQR pyro, comparaison PID/LQR/LQI, balayage Q et R, lieu des pôles, compromis |
| `robustesse_pendule.py` | aller plus loin : bruit, retard, erreur de modèle, saturation, bassin d'attraction, Nyquist |

```bash
pip install numpy scipy matplotlib git+https://github.com/SherbyRobotics/pyro
python partie1_pendule/q3_lqr.py          # figures -> figures/partie1/
```
Compte rendu : [rapport/01](../rapport/01_pendule_PID.md) et [rapport/02](../rapport/02_pendule_LQR.md).
