# Outils communs

| Fichier | Rôle |
|---|---|
| `jumeau.py` | jumeau numérique du SO-101 (`BrasFeetechSimule`, même interface que `feetech_arm`), horloge virtuelle, rendu MuJoCo hors écran → GIF |
| `banc.py` | charge un script original de `partie2_so101/` dans un espace de travail qui imite lerobot-kinematics et remplace écran / horloge / bras / `plt.show` |
| `style.py` | style commun des figures (palette validée pour le daltonisme) |
| `lerobot_shim/` | copie fidèle de `get_robot`, `lerobot_FK`, `lerobot_IK` de lerobot-kinematics sur Robotics Toolbox (pour reproduire sans compiler) |
