# SelfBalancingTetraedre — Robot TétraCubli

Projet de robot auto-équilibrant en forme de tétraèdre de Reuleaux. Le robot repose sur une face, puis utilise l'élan gyroscopique d'une roue de réaction entraînée par un moteur brushless pour se redresser et maintenir sa position verticale en équilibre instable. Le projet regroupe le firmware embarqué (ESP32), des simulations MATLAB pour valider les lois de commande, et des modèles 3D pour la mécanique.

---

## Table des matières

1. [Architecture générale](#architecture-générale)
2. [Matériel utilisé](#matériel-utilisé)
3. [Configuration du projet](#configuration-du-projet)
4. [Code source (`src/`)](#code-source-src)
5. [Simulations MATLAB (`Simu_Matlab/`)](#simulations-matlab-simu_matlab)
6. [Modèles 3D (`modelisation3D/`)](#modèles-3d-modelisation3d)
7. [Configuration VSCode (`.vscode/`)](#configuration-vscode-vscode)
8. [Dépendances (`ESP32Servo`)](#dépendances-esp32servo)
9. [Artefacts de compilation (`.pio/`)](#artefacts-de-compilation-pio)

---

## Architecture générale

```
SelfBalancingTetraedre/
├── src/                    # Code source embarqué (C++)
│   ├── main.cpp
│   ├── Theorie_Proto1FaceBNO.cpp
│   ├── testCapteur.cpp
│   ├── testMoteur.cpp
│   └── testled.cpp
├── Simu_Matlab/            # Simulations de la dynamique (MATLAB)
│   ├── 2D/
│   │   ├── nominale2D.m
│   │   └── redressement2D.m
│   └── 3D/
│       ├── nominale3D.m
│       ├── redressement3D.m
│       ├── exp3.m
│       └── solution.m
├── modelisation3D/         # Fichiers de conception mécanique
│   ├── Cubli 3D.f3d
│   ├── Support moteur.3mf
│   └── Tétracubli.3mf
├── include/                # Fichiers d'en-tête partagés
├── lib/                    # Bibliothèques privées du projet
├── .vscode/                # Configuration de l'éditeur
├── platformio.ini          # Configuration de compilation PlatformIO
└── .gitignore
```

---

## Matériel utilisé

| Composant | Rôle |
|---|---|
| **ESP32** | Microcontrôleur principal (240 MHz, dual-core) |
| **MPU-6500 / MPU-9250** | IMU 6/9 axes (accéléromètre + gyroscope) via I2C |
| **BNO055** | IMU 9 axes avec fusion capteur intégrée (implémentation alternative) |
| **ESC (Electronic Speed Controller)** | Contrôleur de vitesse pour moteur brushless, commande PWM |
| **Moteur brushless** | Entraîne la roue de réaction pour le contrôle gyroscopique |
| **LEDs (GPIO 2, 4, 5)** | Indicateurs de diagnostic |

---

## Configuration du projet

### `platformio.ini`

Fichier de configuration central de PlatformIO qui définit l'environnement de compilation et de téléversement.

- **Plateforme** : `espressif32` — outillage pour microcontrôleurs Espressif ESP32
- **Carte** : `esp32doit-devkit-v1` — variante spécifique avec ses configurations de broches
- **Framework** : `arduino` — permet d'utiliser les API Arduino sur l'ESP32
- **Vitesse du moniteur série** : 115 200 bauds, standard pour la communication de débogage
- **Filtre de compilation** : seul `src/main.cpp` est compilé, pour éviter de compiler les fichiers de test lors du déploiement
- **Dépendance** : `madhephaestus/ESP32Servo @ ^3.0.5`, bibliothèque de contrôle PWM pour servos et ESC

### `.gitignore`

Exclut du suivi Git les fichiers générés automatiquement :
- Le dossier `.pio/` contenant les binaires compilés, fichiers objets et firmwares — inutile à versionner
- Les fichiers de cache IntelliSense et les configurations auto-générées de VSCode (`.vscode/c_cpp_properties.json`, `.vscode/launch.json`, `.vscode/ipch`) pour éviter des conflits entre développeurs

---

## Code source (`src/`)

### `main.cpp` — Contrôleur principal d'équilibre

Programme de production qui implémente la logique complète de redressement et d'équilibrage via une machine à états.

**Capteurs et actionneurs :**
- IMU **MPU-6500** (I2C sur GPIO 21/22 à 400 kHz) pour estimer l'angle et la vitesse angulaire
- **ESC** sur GPIO 18, commandé en PWM à 250 Hz (impulsions entre 1 000 µs et 2 000 µs)

**Algorithmes clés :**

1. **Calibration du gyroscope** : au démarrage, 200 lectures à l'arrêt permettent d'estimer le biais du gyroscope, soustrait ensuite à chaque mesure pour limiter la dérive.

2. **Calibration de la verticale** : l'utilisateur maintient le robot vertical pendant 2 secondes ; la moyenne de 50 mesures définit l'angle de référence `vertical_zero_offset_rad`.

3. **Filtre complémentaire** : fusionne les données accéléromètre (stabilité long-terme) et gyroscope (réactivité court-terme) avec un coefficient `alpha = 0.96` pour estimer l'angle `theta` en temps réel.

4. **Machine à états en 4 phases** :
   - `STATE_LYING_FLAT` : le robot est couché. Il attend que l'angle dépasse 30° puis déclenche la montée en vitesse.
   - `STATE_SPIN_UP` : montée en régime progressive sur 1,5 s, de 1 000 µs à 1 800 µs, pour accumuler l'élan gyroscopique sans à-coups.
   - `STATE_IMPULSE` : coupure instantanée de la commande moteur (1 000 µs), permettant au momentum accumulé de faire basculer le robot vers la verticale. Dure 200 ms ou jusqu'à ce que l'angle passe sous 25°.
   - `STATE_BALANCING` : équilibre actif par régulateur PD+W.

5. **Régulateur PD avec compensation de vitesse roue (PD+W)** :
   - Loi de commande : `τ = -Kp·θ - Kd·θ̇ - Kw·ω_w`
   - `Kp = 0.74` (action proportionnelle à l'angle), `Kd = 0.06` (amortissement), `Kw = 0.005` (compensation de la vitesse roue)
   - La vitesse de la roue `ω_w` est estimée par simulation (encodeur virtuel) puisqu'aucun encodeur physique n'est disponible

6. **Conversion couple → commande PWM** : le couple commandé est converti en courant via la constante moteur `Kt = 0.0095`, saturé à ±12 A, puis mappé en microsecondes pour l'ESC.

**Fréquences d'exécution** : estimation IMU à 200 Hz, boucle de commande à 100 Hz, affichage série à 10 Hz.

---

### `Theorie_Proto1FaceBNO.cpp` — Implémentation alternative avec BNO055

Implémentation alternative utilisant l'IMU **BNO055** d'Adafruit à la place du MPU-6500. Le BNO055 intègre un filtre de Kalman matériel et retourne directement des angles d'Euler fusionnés, simplifiant considérablement le code.

**Différences clés par rapport à `main.cpp` :**
- Pas de filtre complémentaire manuel : le capteur gère lui-même la fusion
- Filtre passe-bas exponentiel appliqué à l'angle de tangage (`alpha = 0.85`)
- Régulateur PD simplifié : `correction = Kp·(pitch_cible - pitch_filtré) - Kd·gyro_z` avec `Kp = 12.0`, `Kd = 5.0`
- Boucle de commande à 200 Hz, correction saturée à ±300 µs autour du point neutre 1 500 µs
- Protection contre les mesures invalides (NaN) : coupure immédiate du moteur si détectée
- ESC sur GPIO 18 à 50 Hz (au lieu de 250 Hz dans `main.cpp`)

Ce fichier représente une voie d'exploration avec un capteur plus intégré, permettant de valider si la fusion matérielle du BNO055 apporte un avantage par rapport à la fusion logicielle du MPU.

---

### `testCapteur.cpp` — Test et validation de l'IMU MPU-9250

Programme de diagnostic permettant de valider le bon fonctionnement de l'IMU et d'explorer les techniques de fusion de capteurs avant de les intégrer au contrôleur principal.

**Fonctionnement :**
- Lit le registre `WHO_AM_I` (0x75) pour identifier le capteur (codes acceptés : 0x71, 0x73, 0x70)
- Effectue une calibration sur 500 échantillons à l'arrêt, en soustrayant 1g sur l'axe Z pour isoler le biais réel
- Applique un filtre complémentaire (alpha = 0.98) pour produire des angles `angle_x` et `angle_y` stables
- Convertit les valeurs brutes 16 bits en unités physiques : accélération en g (÷ 16 384), vitesse angulaire en °/s (÷ 65,5)
- Affiche les données à 10 Hz sur la liaison série

Utile en phase de développement pour vérifier l'orientation du capteur, la qualité du signal, et régler les paramètres de filtrage avant de les transposer dans le contrôleur.

---

### `testMoteur.cpp` — Test et mise en service de l'ESC

Programme de mise en service permettant de vérifier le bon fonctionnement de l'ESC et du moteur brushless, et d'armer l'ESC de manière sécurisée.

**Séquence exécutée :**
1. Allocation des 4 timers matériels PWM de l'ESP32 via la bibliothèque ESP32PWM
2. Envoi de la commande minimale (1 000 µs) pendant 7 secondes pour l'armement de l'ESC — l'utilisateur peut connecter la batterie en toute sécurité pendant ce délai
3. Montée progressive de 1 000 µs à 1 200 µs par incréments de 5 µs toutes les 20 ms (rampe de 4 s)
4. Maintien à 1 200 µs pendant 1 s
5. Descente progressive de 1 200 µs à 1 000 µs au même rythme
6. Pause de 2 s puis répétition du cycle

Ce fichier est exclusivement un outil de développement/commission, pas une partie du firmware de production.

---

### `testled.cpp` — Test des sorties GPIO

Utilitaire minimal de validation des broches GPIO connectées à des LEDs (GPIO 2, 4, 5).

Pour chaque broche, le programme effectue alternativement des tests en logique active-haute (HIGH = LED allumée) et active-basse (LOW = LED allumée), à raison de 4 basculements de 400 ms chacun. Cela permet de vérifier les deux configurations de câblage (alimentation commune vs masse commune) et de s'assurer que le microcontrôleur contrôle bien ses sorties numériques.

---

## Simulations MATLAB (`Simu_Matlab/`)

Les simulations modélisent le tétraèdre de Reuleaux comme un **pendule inversé** et valident la faisabilité du contrôle avant implémentation matérielle. Toutes utilisent l'intégration numérique d'Euler à pas de 5 ms.

### `2D/nominale2D.m` — Stabilisation nominale 2D

Modélise une face unique du tétraèdre traitée comme un pendule inversé 2D. Paramètres : masse 0,25 kg, bras de levier 50 mm, inertie 0,003 kg·m².

Valide que le régulateur **PID** (Kp = 7, Ki = 0,3, Kd = 1,8) avec saturation du couple à ±0,08 N·m stabilise une perturbation initiale de **5°** en moins de 3 s. C'est le cas de base, le plus simple, servant de référence pour les cas plus difficiles.

### `2D/redressement2D.m` — Redressement depuis la position couchée (2D)

Simule le défi fondamental : redresser le robot depuis une position horizontale (70,5° de la verticale).

Stratégie en deux phases :
1. **Phase d'impulsion (0–500 ms)** : couple maximum constant (0,12 N·m, correspondant à la spécification EMAX réelle) pour vaincre la gravité aux grands angles
2. **Phase PID** : régulation fine une fois approché de la verticale

La simulation calcule et affiche la comparaison entre le couple gravitationnel maximum (≈ 0,70 N·m) et le couple moteur disponible, répondant à la question essentielle : **le moteur est-il suffisamment puissant ?**

### `3D/nominale3D.m` — Stabilisation nominale 3D

Identique à la version 2D mais avec les paramètres du système complet 3D : masse 0,5 kg, bras de levier 75 mm, inertie 0,06 kg·m². Les mêmes gains PID sont utilisés, validant leur généralisation à l'échelle 3D avec une perturbation initiale de 5°.

### `3D/redressement3D.m` — Redressement depuis la position couchée (3D)

**Simulation principale** du projet. Valide que le tétraèdre 3D complet (0,5 kg) peut se redresser depuis 70,5° avec le moteur disponible (0,10 N·m). Utilise la même stratégie en deux phases que la version 2D. L'inertie est volontairement réduite (0,006 kg·m²) pour les calculs aux grands angles afin d'éviter des vitesses irréalistes — phénomène bien connu en commande non-linéaire. C'est la preuve de concept centrale : si cette simulation échoue, la conception matérielle est infaisable.

### `3D/exp3.m` — Effet de la géométrie courbe (surface de Reuleaux)

Explore une caractéristique unique du tétraèdre de Reuleaux : ses faces sont **courbes**, pas planes. Quand le robot se penche sur une face courbe, le point de contact se déplace et le bras de levier effectif varie selon la relation `lb = 0,075 × cos(θ/2)`.

Cette non-linéarité géométrique supplémentaire est testée avec une perturbation initiale de 15° pour observer son effet sur la dynamique. La simulation détermine si la courbure naturelle est plutôt un avantage (réduction du couple déstabilisant) ou une complication pour le contrôle.

### `3D/solution.m` — Redressement par impulsion de roue de réaction

Simule un mécanisme alternatif de redressement : une **roue de réaction** (80 g, rayon 30 mm) tournant à 8 000 tr/min est brusquement freinée, transférant son moment cinétique au corps du robot pour initier le redressement.

La quantité de mouvement angulaire transférée génère une vitesse initiale calculée, affichée en console ("Vitesse de remontée générée par le choc"). Après ce transfert impulsionnel, le régulateur PID prend le relais pour maintenir l'équilibre. Cette approche s'inspire des systèmes de contrôle d'attitude des satellites. La simulation révèle que l'impulsion générée reste modeste à cette échelle, faisant du couple moteur continu le mécanisme principal de redressement.

---

## Modèles 3D (`modelisation3D/`)

### `Cubli 3D.f3d`

Fichier Fusion 360 (`.f3d`) contenant la conception CAO paramétrique complète du robot. Permet de modifier les dimensions, vérifier les assemblages, réaliser des analyses d'interférence, et préparer les exports pour la fabrication.

### `Support moteur.3mf`

Fichier d'impression 3D (format `.3mf`) pour la pièce de fixation du moteur. Ce support assure le positionnement précis et rigide du moteur brushless dans le tétraèdre. Le format `.3mf` (successeur du STL) inclut les informations de couleur, matériau et orientation d'impression.

### `Tétracubli.3mf`

Fichier d'impression 3D pour le corps principal du robot. Contient la géométrie du tétraèdre de Reuleaux avec ses faces caractéristiques courbées, conçu pour être imprimé puis assemblé avec les composants électroniques et mécaniques.

---

## Configuration VSCode (`.vscode/`)

### `c_cpp_properties.json`

Configuration auto-générée par PlatformIO pour l'IntelliSense C++ de VSCode. Définit plus de 240 chemins d'inclusion couvrant l'intégralité du framework Arduino, les bibliothèques ESP32 IDF (FreeRTOS, SPI, I2C, Bluetooth, réseau), et les dépendances du projet. Configure le compilateur Xtensa GCC, les standards C99/C++11 avec extensions GNU, et les macros de préprocesseur (`F_CPU = 240000000L`, `ARDUINO_ARCH_ESP32`, etc.). Permet l'autocomplétion et la navigation dans tout le code framework directement dans l'éditeur.

### `extensions.json`

Recommande l'extension **PlatformIO IDE** (`platformio.platformio-ide`) et déconseille explicitement le pack C++ de Microsoft (`ms-vscode.cpptools-extension-pack`) qui entrerait en conflit avec la gestion IntelliSense de PlatformIO. Assure un environnement cohérent pour tous les développeurs du projet.

### `launch.json`

Fournit trois configurations de débogage matériel via OpenOCD :
- **PIO Debug** : compilation puis débogage (workflow standard)
- **PIO Debug (skip Pre-Debug)** : débogage sans recompilation (itération rapide)
- **PIO Debug (without uploading)** : débogage sans réécriture du firmware (debug à distance ou firmware déjà en place)

### `settings.json`

Désactive les soulignements d'erreur rouge (`C_Cpp.errorSquiggles = "disabled"`) pour éviter les doublons entre l'analyseur PlatformIO et celui de Microsoft CppTools.

---


## Dépendances (`ESP32Servo`)

La bibliothèque **ESP32Servo v3.1.3** (installée automatiquement dans `.pio/libdeps/`) fournit le contrôle PWM précis nécessaire pour commander l'ESC.

**Caractéristiques clés :**
- Utilise le périphérique **LEDC** (LED Control) de l'ESP32 pour générer des signaux PWM par hardware — beaucoup plus précis qu'une solution logicielle
- 16 canaux PWM disponibles sur l'ESP32 standard
- Supporte les commandes en microsecondes directes avec résolution 16 bits
- Plage de commande : 500–2 500 µs
- Allocation automatique des ressources hardware, sans configuration manuelle des timers

Dans ce projet, elle est utilisée exclusivement pour contrôler l'ESC du moteur brushless via des impulsions PWM calibrées (1 000–2 000 µs).

---

## Artefacts de compilation (`.pio/`)

Répertoire auto-généré par PlatformIO, non versionné (exclu par `.gitignore`).

- **`.pio/build/esp32doit-devkit-v1/`** : contient `firmware.bin` (image à flasher sur l'ESP32), `firmware.elf` (avec symboles de débogage), `firmware.map` (carte mémoire), `bootloader.bin`, `partitions.bin`, et tous les fichiers objets intermédiaires
- **`.pio/libdeps/esp32doit-devkit-v1/`** : dépendances téléchargées et compilées automatiquement (ESP32Servo)
- **`idedata.json`** : données d'environnement exportées vers VSCode pour alimenter l'IntelliSense

Ces fichiers sont régénérés à chaque compilation et ne doivent pas être modifiés manuellement.

---

## Démarrage rapide

1. Installer [PlatformIO](https://platformio.org/) via l'extension VSCode
2. Ouvrir le dossier du projet — PlatformIO détecte `platformio.ini` automatiquement
3. Brancher l'ESP32 en USB
4. Compiler et téléverser avec le bouton **Upload** de PlatformIO (ou `pio run -t upload`)
5. Ouvrir le moniteur série à 115 200 bauds pour suivre le démarrage et la calibration
6. Maintenir le robot vertical pendant 2 s lors de la phase de calibration initiale

Pour tester les composants individuellement, modifier temporairement `build_src_filter` dans `platformio.ini` pour cibler `testCapteur.cpp`, `testMoteur.cpp`, ou `testled.cpp`.
