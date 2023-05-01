# BE Filtrage optimal et compte rendu du travail effectué

## Introduction
Ce dossier contient les codes produits faisant appel au filtrage de Kalman ainsi que les slides présentés dans les vidéos.
En effet vous trouverez ci dessous les liens vers des vidéos présentant le travail effectué.

## Videos
### 1. Introduction à la coupe de France de Robotique 
[Lien vers la vidéo](https://polymny.studio/v/3DejV/)
Dans cette vidéo vous trouverez une présentation de la coupe de France de robotique ainsi que des différents défis qui sont a relever cette année. De plus l'architecture logiciel sera ici présentée afin de donner du contexte aux vidéos suivantes.

### 2. Positionnement lidar
[A venir](...)
Cette vidéo présente le fonctionnement du système permettant de placer le robot et des potentiels obstacles sur le plateau de jeu.

### 3. Fusion des capteurs et Estimation de la position des Ennemis
[A venir](...)
Ici nous présenterons les différents algorithmes se basant sur le filtrage de Kalman permettant dans un premier temps d'obtenir une estimation plus précise et moins bruitée de la position du robot. Et dans un second temps d'estimer la position des ennemis (obstacles) présents sur le plateau de jeu.

### 4. Noeud de Navigation
[Lien vers la vidéo](https://drive.google.com/file/d/1D4rdq3Z5j8yqn6Kk88j0iWKAar3zTYeO/view)
Ici sera présenté le noeud de navigation qui permet au robot de se déplacer sur le plateau de jeu de manière rapide tout en évitant les obstacles.

## Slides 
Les slides présentés dans les vidéos sont disponibles en pdf à la racine de ce dossier. De plus afin de pouvoir visualiser les gifs présents sur les transparents, vous pouvez accèder à la visionneuse en ligne en cliquant sur le lien suivant : [Visionneuse](https://docs.google.com/presentation/d/e/2PACX-1vRQYQfNccN41Cf5IWjL6nRUtd15CT7ILjTp0NbM19YL2TLo_ZCS5epXIBjujxFY5e2HkT87-7fDsdKH/pub?start=true&loop=false&delayms=60000)

## Notes pour le code
Deux fichiers python sont présents dans le dossier `code`:
- `Kalman_filter_ennemis.py` : Ce fichier contient le code permettant de filtrer la position des ennemis sur le plateau de jeu.

- `Kalman_filter_robot_method_1.py` : Ce fichier contient le code permettant de filtrer la position du robot sur le plateau de jeu en utilisant une première méthode qui consiste à utiliser l'odométrie pour faire de la prédiction.

- `Kalman_filter_robot_method_2.py` : Ce fichier contient le code permettant de filtrer la position du robot sur le plateau de jeu en utilisant une deuxième méthode qui en plus d'un prédiction odométrique, se repose sur une loi a priori suivant les lois du mouvements.

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


