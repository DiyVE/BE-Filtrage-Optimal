# BE Filtrage optimal et compte rendu du travail effectué

## Introduction
Dans ce dossier vous trouverez tout d'abord plusieurs vidéos expliquant les différentes composantes du système de navigation utilisé dans robot participant a la coupe de France de robotique 2023. De plus vous retrouverez les slides des présentées dans les videos ainsi que les codes correspondant au filtrage de Kalman.

## Videos
### 1. Introduction à la coupe de France de Robotique 
Dans cette vidéo vous trouverez une présentation de la coupe de France de robotique ainsi que des différents défis qui sont a relever cette année. De plus l'architecture logiciel sera ici présentée afin de donner du contexte aux vidéos suivantes.

### 2. Positionnement lidar
Cette vidéo présente le fonctionnement du système permettant de placer le robot et des potentiels obstacles sur le plateau de jeu.

### 3. Fusion des capteurs et Estimation de la position des Ennemis
Ici nous présenterons les différents algorithmes se basant sur le filtrage de Kalman permettant dans un premier temps d'obtenir une estimation plus précise et moins bruitée de la position du robot. Et dans un second temps d'estimer la position des ennemis (obstacles) présents sur le plateau de jeu.

### 4. Noeud de Navigation  
Ici sera présenté le noeud de navigation qui permet au robot de se déplacer sur le plateau de jeu de manière rapide tout en évitant les obstacles.

## Notes pour le code
Deux fichiers python sont présents dans le dossier `code`:
- `Filtrage_position_ennemis` : Ce fichier contient le code permettant de filtrer la position des ennemis sur le plateau de jeu.
- `Filtrage_position_robot` : Ce fichier contient le code permettant de filtrer la position du robot sur le plateau de jeu.

### 1. Installation des dépendances
Afin de simplifier l'installation des dépendances, un fichier `requirements.txt` est présent dans le dossier `code`. Pour installer les dépendances, il suffit de se placer dans le dossier `code` et d'exécuter la commande suivante :
```bash
pip3 install -r requirements.txt
```
 - Numpy
 - Matplotlib
 - Rosbags

### 2. Lancement des scripts
Pour lancer les scripts, il faut se placer dans le dossier `code` et exécuter la commande suivante :
```bash
python3 <nom_du_script>.py
```


