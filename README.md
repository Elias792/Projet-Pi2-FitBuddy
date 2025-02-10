# FitBuddy – Prototype (Priorité 1)

Ce dépôt présente le **projet PI²** de notre équipe, dédié à la conception d’un prototype de capteur FitBuddy (version Priorité 1).  
Il couvre principalement :

- La **lecture** de l’IMU (LSM6DSOX) pour récupérer l’accélération et la rotation,  
- La **détection** des répétitions (nombre, temps par répétition, vitesse, amplitude, etc.),  
- Un **exemple** de transmission via BLE (service GATT simplifié).  

> **Important :** Il s’agit d’un prototype minimal pour la **Priorité 1**.  
> Pour une version complète, il faudra inclure la gestion BLE Mesh, UWB, alimentation par batterie, etc.

---

## Table des Matières

1. [Contexte du Projet](#contexte-du-projet)  
2. [Structure du Dépôt](#structure-du-dépôt)  
3. [Prérequis](#prérequis)  
4. [Compilation et Flash](#compilation-et-flash)  
5. [Test et Utilisation](#test-et-utilisation)  
6. [Fonctionnalités Priorité 1](#fonctionnalités-priorité-1)  
7. [Évolutions Possibles](#évolutions-possibles)  
8. [Dépôt sur GitHub](#dépôt-sur-github)  
9. [Crédits et Contact](#crédits-et-contact)

---

## Contexte du Projet

Dans le cadre du projet **PI²**, notre équipe développe un capteur capable de s’adapter à diverses machines de musculation.  
**Objectif** : suivre les performances des utilisateurs en temps réel (répétitions, vitesse, temps de pause, etc.).

Ce prototype (Pré-version) se concentre sur :
1. La **lecture** des données d’accélération (±2g à ±16g) et éventuellement de gyroscope,  
2. La **détection** basique des répétitions,  
3. L’**exemple** de partage des données (BLE GATT, logs console).

---

## Structure du Dépôt.
├── CMakeLists.txt
├── prj.conf
├── README.md        
└── src/
    ├── main.c
    ├── sensor_lsm6dsox.c
    ├── sensor_lsm6dsox.h
    ├── rep_detection.c
    ├── rep_detection.h
    └── ble_service.c


- **`main.c`** : Point d’entrée du firmware (initialisation IMU, boucle principale).  
- **`sensor_lsm6dsox.[ch]`** : Gestion de l’IMU (I2C, configuration, lecture).  
- **`rep_detection.[ch]`** : Algorithme simple pour détecter les répétitions, mesurer le temps par répétition, etc.  
- **`ble_service.[ch]`** : Exemple de service BLE minimal (GATT) pour la transmission.

---

## Prérequis

- **Carte** : nRF5340 DK (ou autre carte basée sur nRF5340).  
- **SDK** : nRF Connect SDK / Zephyr (version 2.x ou supérieure).  
- **Outils** :  
  - `west` pour la compilation et le flash,  
  - Un logiciel de log (JLink RTT Viewer, minicom, etc.) pour visualiser les `LOG_INF`.  
- **Capteur** : LSM6DSOX (branché en I2C ou I3C, adresse et broches à ajuster dans le code).

---

## Compilation et Flash

1. **Cloner le dépôt** (en local) :  
   ```bash
   git clone https://github.com/Elias792/Projet-Pi2.git
   cd Projet-Pi2

