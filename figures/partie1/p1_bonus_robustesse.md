| Contrôleur | nominal | bruit 0.3° | retard 10 ms | retard 25 ms | ωg² réel = 48 | u max 15 rad/s² | calage 1° |
|---|---|---|---|---|---|---|---|
| PID Shawn | butée rotor | butée rotor | butée rotor | tombe | butée rotor | butée rotor | butée rotor |
| Double PID | 0.68° θmax 17° | 0.69° θmax 17° | 0.74° θmax 17° | 1.05° θmax 20° | 0.91° θmax 22° | 0.68° θmax 17° | 0.80° θmax 16° |
| LQR | 0.51° θmax 17° | 0.58° θmax 16° | 0.52° θmax 17° | 0.54° θmax 16° | 0.60° θmax 21° | 0.53° θmax 18° | 0.62° θmax 26° |
| LQI | 0.59° θmax 14° | 0.69° θmax 14° | 0.60° θmax 14° | 0.61° θmax 13° | 0.68° θmax 17° | 0.62° θmax 15° | 0.72° θmax 19° |

Retard maximal toléré (ms) : Double PID 30.0, LQR 60.0, LQI 50.0

Bassin d'attraction (°) : PID Shawn 5.9, Double PID 19.2, LQR 24.1, LQI 22.6

Bassin LQR vs R : R=0.01: 16.1°, R=0.1: 20.6°, R=1: 24.1°, R=10: 24.1°, R=100: 18.7°

Double PID (sans filtres) : marge de phase 27° à ωc = 10.8 rad/s -> marge de retard ≈ 44 ms

LQR nominal : marge de phase 60° à ωc = 17.7 rad/s -> marge de retard ≈ 59 ms
