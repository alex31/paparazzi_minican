# Procédure de test des rôles MicroCAN V5

Public : stagiaire de première année de BTS. Version de la procédure : 25 septembre 2026.

**Objectif : vérifier chaque rôle séparément, depuis son branchement jusqu'au résultat physique et aux messages CAN.** Le projet décrit 15 rôles fonctionnels (dont le shell), un mode d'identification et un modèle de développement, soit 17 rôles. Le modèle `ROLE.template` est exclu de la compilation habituelle ; `ROLE.shell` est absent des versions `NOSHELL=1`.

Cette procédure est fondée sur les sources du projet et des outils disponibles. Elle ne constitue pas un compte rendu d'essais matériels déjà réalisés. Remplir la fiche de résultats en fin de document pendant les manipulations. Une fonction sans matériel disponible est « non testée », jamais « OK ».

Parcours : préparer le banc (§1–2), suivre les fiches T01 à T17 (§3), effectuer les contrôles complémentaires (§4), puis remettre le banc en état et compléter les résultats (§7). Les références T01 à T17 permettent de retrouver un rôle avec la recherche du lecteur Markdown.

## 1. Comprendre le banc

| Terme | Signification pour ce TP |
| --- | --- |
| Rôle | Fonction que l'on active dans la carte, par exemple lire un GPS. |
| Paramètre | Réglage de la carte ; `ROLE.sbus=true` active la réception SBUS au prochain démarrage. |
| Nœud / Node ID | Appareil sur le CAN et son numéro ; le PC possède aussi un numéro lorsqu'il émet. |
| Message / publication | Donnée envoyée sur le CAN, par exemple une distance en mètres. |
| Télémétrie | Mesure renvoyée par un appareil, par exemple le régime du moteur. |
| UART | Liaison série avec TX (émission), RX (réception) et GND (masse). |
| I2C | Liaison de capteurs avec SDA (données), SCL (horloge) et GND. |
| PWM | Impulsions dont la durée commande un servo. |
| `NaN` | Mesure indisponible ; ce n'est pas la valeur zéro. |
| `true` / `false` | Activé / désactivé. |

Le montage commun est le suivant :

```text
PC Linux -- USB -- adaptateur USB-CAN -- CAN_H / CAN_L / GND -- MicroCAN
                                                               |
                                                    périphérique du rôle testé

Alimentation de laboratoire -------------------------- VBUS / GND
```

Utiliser **une seule MicroCAN et aucun aéronef connecté**. Les commandes de servos, d'ESC et de LED sont diffusées à tout le bus : choisir un « Target node » dans un outil ne transforme pas ces messages en commandes privées.

### Matériel commun

- PC Linux avec DroneCAN GUI Tool, les outils du dossier `../TOOLS` et les droits d'accès CAN/série.
- Adaptateur USB-CAN compatible SocketCAN, par exemple un CANable configuré pour présenter `can0`.
- Alimentation réglable avec limitation de courant, multimètre et cordons.
- Adaptateur USB-UART **TTL 3,3 V**, utile pour la console de dépannage et les essais série.
- Oscilloscope ou analyseur logique pour les sorties PWM ; sondes adaptées aux niveaux mesurés.
- Les périphériques indiqués dans chaque fiche.

Les applications doivent être installées et les exécutables compilés avant le TP. Le tuteur prépare les pilotes et dépendances manquants ; les commandes de compilation figurent en annexe.

### Alimentation et branchements

1. Couper l'alimentation avant de déplacer les fils ou de changer de périphérique.
2. Vérifier le sens des connecteurs sur la carte et le schéma, pas seulement la couleur des fils.
3. Pour la carte seule, utiliser par exemple **12 V sur VBUS**, avec une limitation initiale de **0,2 A**. Si l'alimentation limite le courant, couper et vérifier le montage avant de l'augmenter.
4. Le schéma indique `6 V < VBUS < 26 V`. Ces limites concernent **VBUS**, jamais les broches de signal, les sorties 3,3 V ou les sorties 5 V.
5. Relier les masses du PC/adaptateur, de la carte et des alimentations des périphériques. Ne pas relier entre elles les sorties positives de deux alimentations.
6. Alimenter servos et moteurs par une alimentation adaptée à leur fiche technique. Pour ce TP, utiliser une alimentation séparée de puissance et une masse commune.
7. Pour tout essai moteur : **retirer l'hélice, fixer le moteur et garder la coupure de puissance accessible**. Pour les servos : retirer les tringleries et dégager la course.
8. Utiliser des modules de capteurs avec alimentation et niveaux logiques compatibles. Une puce nue et un module avec régulateur n'ont pas nécessairement les mêmes tensions admissibles.

Le document matériel [MicroCan_v5_schematic.pdf](HARDWARE/MICROCAN/MicroCan_v5_schematic.pdf) porte le titre « MiniCAN v5 ». Vérifier avec le tuteur qu'il correspond bien à la carte du banc ; les signaux ci-dessous sont aussi recoupés avec [MICROCAN.cfg](microcan/cfg/MICROCAN.cfg).

### Repérage des connecteurs

Les numéros sont ceux du schéma, **pas un ordre gauche-droite supposé en regardant la carte**.

| Connecteur | Broches utiles |
| --- | --- |
| J1 / J2, CAN | 1 : GND ; 2 : VBUS ; 3 : CAN_H ; 4 : CAN_L. |
| J4, UART applicatif | 1 : GND ; 2 : +5 V ; 3 : +3,3 V ; 4 : RX/PB04 ; 5 : TX/PB03. |
| J5, I2C | 1 : GND ; 2 : +5 V ; 3 : +3,3 V ; 4 : SDA/PB07 ; 5 : SCL/PA15. |
| J6, SPI | 1 : GND ; 2 : +5 V ; 3 : +3,3 V ; 4 : CS/PA04, aussi entrée microphone et PWM CH6. |
| J7, servos | 1 : GND ; 2 : +5 V ; 3 : +3,3 V ; 4 : CH1/PA08 ; 5 : CH2/PA09 ; 6 : CH3/PA10 ; 7 : CH4/PA11. |
| J3, PROBE | 1 : GND ; 6 : DBG_TX/PA02 ; 7 : DBG_RX/PA03. |

Les canaux PWM supplémentaires sont **CH5 = J4-4/PB04**, **CH6 = J6-4/PA04** et **CH7 = J5-4/PB07**. Ils réutilisent les connecteurs UART, SPI et I2C.

## 2. Préparer les logiciels et accéder à la carte

### 2.1. Ouvrir les bons dossiers

Toutes les commandes Linux de ce document partent de la racine du projet :

```sh
cd ~/DEV/STM32/CHIBIOS/C21DEV/MINICAN/MICROCAN_V5
mkdir -p resultats_tests
```

Il existe deux dossiers différents :

| Dossier | Contenu utilisé dans ce TP |
| --- | --- |
| `../TOOLS` | `minican_tester`, `uavcan_logger`, `uavcan_param_editor`, `uavcan_replay`, `uavcan_fw_updater`. |
| `tools` | Petits scripts locaux, dont `send_can_file.pl` et les générateurs DSDL. |

`../TOOLS` signifie « remonter d'un dossier, puis entrer dans TOOLS ». Linux distingue majuscules et minuscules.

### 2.2. Mettre le CAN en service

Le firmware standard de ce projet utilise le CAN classique à **1 000 000 bit/s** (`CAN_BITRATE=1000` à la compilation). Vérifier la version du firmware avec le tuteur ; une autre compilation peut utiliser un autre débit ou CAN FD.

Brancher CAN_H sur CAN_H, CAN_L sur CAN_L et GND sur GND. Ne pas alimenter VBUS depuis la broche 5 V d'un adaptateur USB-CAN. Il faut **une terminaison de 120 ohms à chaque extrémité**, donc deux au total. `can.terminal_resistor=true` active celle de la MicroCAN au démarrage normal. Si deux terminaisons externes sont déjà installées, désactiver celle de la carte.

Dans un terminal Linux, pour un adaptateur qui fournit déjà `can0` :

```sh
ip link show can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

Si `can0` n'existe pas, faire configurer l'adaptateur par le tuteur. Un adaptateur SLCAN ouvert directement comme `/dev/ttyACM0` dans la GUI ne fournit pas automatiquement `can0` aux outils C++ de `TOOLS`.

`restart-ms 100` demande une reprise automatique après un état CAN `BUS-OFF` ; cela peut être utile lorsque la seule carte du banc redémarre ou quitte temporairement le bus. Des erreurs persistantes nécessitent malgré tout une vérification du câblage et du débit.

### 2.3. Sortir du mode identification, si nécessaire

**Sur une configuration neuve, `ROLE.identification` vaut `true`. Avec le firmware corrigé, la LED est violette et le nœud CAN reste accessible en mode `MAINTENANCE`.** Les autres rôles ne sont pas démarrés, sauf le shell si `ROLE.shell=true`. Ouvrir la GUI comme au §2.4, activer l'allocation dynamique si nécessaire, puis régler `ROLE.identification=false`, cliquer **Store All** et **Restart** pour commencer les essais de rôles.

**Cas d'un ancien firmware :** avant le correctif, le mode identification empêchait le démarrage du CAN. Une carte déjà bloquée avec cette version ne peut pas recevoir la correction par un bus qu'elle n'écoute pas. Utiliser la console ci-dessous pour désactiver ce mode, puis charger le firmware corrigé. Si la console est absente, utiliser une interface de programmation filaire avec le tuteur.

La console de dépannage se branche ainsi :

| Adaptateur USB-UART TTL 3,3 V | MicroCAN J3 PROBE |
| --- | --- |
| TX | Broche 7, DBG_RX/PA03 |
| RX | Broche 6, DBG_TX/PA02 |
| GND | Broche 1, GND |

Ne pas brancher le +5 V de l'adaptateur. La carte reste alimentée par VBUS. Ouvrir un terminal série à **115200 bit/s, 8 bits, sans parité, 1 bit d'arrêt, sans contrôle de flux**. Ce connecteur de diagnostic est distinct de J4 et de l'UART utilisé pour le flash initial.

Dans la **console de la carte**, saisir une commande par ligne :

```text
st ROLE.identification
st ROLE.identification false
st uavcan.node_id 10
st uavcan.param_set_behavior 1
restart
```

`st nom valeur` écrit et sauvegarde le paramètre. Le numéro 10 est réservé ici à l'unique carte du banc. Si elle avait une configuration utile, relever les anciennes valeurs avant de les modifier. Une carte qui ne démarre plus un rôle se récupère aussi avec `st ROLE.nom_du_role false`, puis `restart`.

Si la console n'apparaît pas, vérifier les broches, TX/RX et la version du firmware avec le tuteur. Dans le nouveau firmware, il faut **`ROLE.shell=true`**, sauvegardé puis suivi d’un redémarrage ; cela fonctionne aussi en identification (fiche T17). Le shell est désactivé par défaut. Une compilation **`NOSHELL=1`** retire ce rôle et son paramètre ; le brancher ne suffit donc pas à le lancer. Le firmware corrigé ne dépend pas de cette console pour configurer la carte ou mettre à jour son firmware par CAN.

### 2.4. Démarrer DroneCAN GUI Tool

Dans un **terminal Linux**, lancer la version préparée pour ce projet :

```sh
dronecan_gui_tool --interface can0 --bitrate 1000000 \
  --dsdl "$HOME/DEV/STM32/UAVCAN/DSDL/microcan"
```

Le nom de l'exécutable est `dronecan_gui_tool`. Le chemin `--dsdl` permet de décoder les messages microphone et lumière. Les définitions partagées sont dans `~/DEV/STM32/UAVCAN/DSDL`, branche **minican** ; ne pas en recopier une version dans ce projet. Si la commande n'est pas disponible, faire activer l'environnement logiciel du banc.

1. Dans **Local node properties**, choisir le numéro **126**, puis **Set**. C'est le numéro du PC, pas celui de la carte.
2. Laisser **Send CANFD** désactivé pour ce banc en CAN classique.
3. Vérifier que la MicroCAN apparaît avec le numéro **10**. Son temps de fonctionnement, `uptime`, doit augmenter.
4. Si le tuteur a conservé `uavcan.node_id=0`, lancer le **Dynamic node ID allocation server** dans la GUI, puis attendre l'attribution d'un numéro. Utiliser ce numéro dans les tests à la place de 10. Un seul serveur d'allocation doit être actif.
5. Double-cliquer sur la carte pour ouvrir **Node Properties**. Cliquer sur **Fetch All** dans les paramètres.
6. Avant toute modification, cliquer sur **Save to File** et enregistrer `resultats_tests/parametres_avant_tests.json`.

Les noms de boutons ci-dessus correspondent à la version locale dans `~/DEV/STM32/UAVCAN/gui_tool`. La GUI permet de surveiller, configurer et envoyer des messages depuis une console Python ; voir la [présentation officielle](https://dronecan.github.io/GUI_Tool/Overview/).

### 2.5. Modifier les paramètres pour chaque fiche

Appliquer cette méthode **avant chaque rôle**, même si le rôle précédent a réussi :

1. Arrêter les émissions du test précédent. Pour un moteur, envoyer d'abord la consigne zéro et vérifier son arrêt, puis couper sa puissance.
2. Dans **Fetch All**, mettre `uavcan.param_set_behavior=1` : les changements sont sauvegardés sans redémarrage automatique. La valeur 2 provoquerait des redémarrages pendant la configuration.
3. Pour les fiches T01 à T15 et T17, mettre **tous les `ROLE.*` présents à `false`**, y compris `ROLE.health.survey`, puis activer uniquement celui indiqué par la fiche. `ROLE.identification` reste `false`. **Exception T16 : conserver les rôles et leurs réglages d'une configuration fonctionnelle**, pour vérifier que l'identification les suspend sans les modifier.
4. Saisir les paramètres de la fiche ; utiliser un point pour les décimales, par exemple `0.2`.
5. Valider chaque cellule éditée. **Store All** sauvegarde les paramètres dans la carte ; **Save to File** crée seulement une copie sur le PC.
6. Couper l'alimentation, câbler le périphérique, remettre sous tension. Si le câblage était déjà correct, **Restart** permet un redémarrage logiciel.
7. Attendre le retour de la carte, refaire **Fetch All** et vérifier que les valeurs ont été conservées. Les rôles sont choisis au démarrage.
8. Pour les rôles normaux, vérifier le mode `OPERATIONAL`. S'il reste en initialisation, lire les journaux et vérifier le branchement ; un échec de démarrage d'un capteur peut empêcher la suite du démarrage.

Un paramètre absent peut correspondre à un rôle non compilé ou à un ancien firmware. Ne pas créer un nom approchant : consulter [roleConf.h](COMMON/source/roleConf.h) et le tuteur. `ROLE.template` est normalement absent car son rôle est exclu de la compilation.

### 2.6. Voir les messages et conserver une preuve

Dans la GUI, ouvrir **Tools → Bus Monitor**. Repérer la colonne source (`Src`) et le nom du message. Cliquer sur une ligne pour lire le contenu décodé. **Un message de source 126 est émis par le PC ; une mesure de la carte doit venir de 10.**

Un message peut occuper plusieurs trames CAN : pour vérifier une cadence de mesures, compter les messages complets décodés, pas les lignes de trames brutes.

Pour lire plus facilement un type précis, ouvrir **Tools → Interactive Console**. Les blocs marqués `python` ci-dessous se collent dans cette console, pas dans le terminal Linux. Saisir au début de chaque nouvelle console :

```python
cible = 10
```

Exemple d'observation filtrée sur la carte :

```python
subscribe(dronecan.uavcan.equipment.device.Temperature, lambda e: print_yaml(e.message) if e.transfer.source_node_id == cible else None)
```

Pour arrêter les abonnements et émissions périodiques créés dans cette console :

```python
stop()
```

**`stop()` n'envoie pas de consigne zéro aux moteurs ou aux servos et n'arrête pas les émissions de `minican_tester`.** Utiliser la séquence d'arrêt de leur fiche.

Pour enregistrer les mesures dans un second terminal Linux, remplacer le nom de fichier à chaque rôle :

```sh
../TOOLS/uavcan_logger/uavcan_logger --iface can0 --node-id 124 \
  --dsdl "$HOME/DEV/STM32/UAVCAN/DSDL" \
  --log resultats_tests/health_survey.log
```

Arrêter avec **Ctrl+C**. Le logger charge la racine complète `DSDL`, tandis que l'option GUI ci-dessus charge le namespace supplémentaire `DSDL/microcan`. Vérifier qu'il décode bien `20900` et `20901` pour les deux capteurs concernés.

Numéros réservés sur ce banc : MicroCAN **10**, GUI **126**, tester **125**, logger **124**, éditeur de paramètres **123**. Aucun ne doit être partagé par deux applications actives.

### 2.7. Utiliser `minican_tester`

Lancer depuis la racine du projet :

```sh
../TOOLS/minican_tester/build/minican_tester
```

1. Choisir l'interface `can0`, le nœud cible 10 et le rôle demandé.
2. Choisir l'identifiant local manuel 125 si nécessaire.
3. Pour `SerialStream` et `Futaba SBUS`, sélectionner aussi le port de l'adaptateur USB-UART relié à **J4**, pas celui de la console J3.
4. Cliquer **Prepare Session (Recommended)**. L'outil découvre la carte et lit ses paramètres. Il **ne configure pas les rôles** : les modifier dans la GUI si nécessaire.
5. Vérifier les paramètres affichés. Au besoin, cliquer **Read Device Config** après reconnexion.
6. Pour les essais indiqués, choisir **Duration = 30 s**, **Timeout = 1200 ms**, puis **Start Long Test**.
7. Conserver le journal et une capture des compteurs. **Stop Test** interrompt le scénario ; ce n'est pas une coupure de puissance.

Ne laisser qu'un seul outil émettre les commandes du rôle testé. La GUI peut rester ouverte pour observer, à condition de ne pas avoir de publication ou de panneau de commande actif.

## 3. Fiches de test, rôle par rôle

Ordre conseillé : surveillance, tunnel, SBUS, capteurs, LED, voltmètre, PWM, servo intelligent, DShot, modèle, identification et shell. Pour chaque fiche, appliquer d'abord le §2.5.

### T01 — Surveillance de la carte : `ROLE.health.survey`

**Matériel et branchement :** carte seule alimentée en 12 V sur VBUS, multimètre entre VBUS et GND ; mesurer aussi le 3,3 V sur J5-3 par rapport à J5-1.

**Paramètre :** `ROLE.health.survey=true`.

**Manipulation :**

1. Ouvrir **Panels → CircuitStatus** dans la GUI, ou observer `uavcan.equipment.power.CircuitStatus` dans Bus Monitor.
2. Relever pendant 30 s le circuit 0 (batterie/VBUS) et le circuit 1 (alimentation 3,3 V).
3. Observer `uavcan.equipment.device.Temperature`, `device_id=0`. Convertir en degrés Celsius : **°C = K − 273,15**.
4. Comparer les tensions aux mesures du multimètre. Passer VBUS de 12 V à 10 V, attendre la stabilisation et vérifier que le circuit 0 suit.
5. Remettre 12 V. Enregistrer `health_survey.log`.

**Réussite :** environ une publication par seconde de chaque type/circuit, température plausible pour le microcontrôleur et tensions qui suivent les mesures. Pour ce TP, viser un écart de tension inférieur à **5 %** ; c'est un critère de banc proposé, pas une précision garantie du produit. Le courant est `NaN` : ce rôle ne le mesure pas.

**Si anomalie :** vérifier que VBUS est effectivement alimenté ; une alimentation par la prise de programmation seule ne fournit pas forcément une mesure batterie. Relever les `error_flags`. Ne pas modifier `adc.psbat.scale` et `bias` simplement pour faire réussir le test : la calibration est une opération distincte avec le tuteur.

### T02 — Tunnel série : `ROLE.tunnel.serial`

**Matériel :** adaptateur USB-UART TTL 3,3 V. Brancher **TX adaptateur → J4-4/RX**, **RX adaptateur → J4-5/TX**, GND → J4-1. Laisser son alimentation positive débranchée.

| Paramètre | Valeur |
| --- | --- |
| `ROLE.tunnel.serial` | `true` |
| `bus.serial.baudrate` | `115200` |
| `role.tunnel.serial.protocol` | `0` |
| `role.tunnel.serial.channel_id` | `0` |

**Manipulation :**

1. Dans `minican_tester`, sélectionner **SerialStream**, le port UART, puis **Prepare Session**.
2. Lancer **Start Long Test** pendant 30 s. L'outil envoie des données aléatoires dans les deux sens.
3. Observer `uavcan.tunnel.Broadcast` dans Bus Monitor. Les émissions du PC et de la carte ont des sources différentes.
4. Relever les compteurs `CAN->Serial misses`, `Serial->CAN misses`, les incohérences éventuelles et les temps de réponse.
5. Refaire un essai de 30 s à `bus.serial.baudrate=230400`, après sauvegarde, redémarrage et nouvelle préparation de session.

**Réussite :** données reçues identiques aux données envoyées dans les deux sens, aucun paquet manquant ni incohérence sur les deux essais. Un simple message `Broadcast` visible n'est pas une vérification du contenu.

**Complément manuel, outil local :** fermer le port série dans le tester. Dans un terminal Linux, remplacer `/dev/ttyUSB0` par le bon port :

```sh
stty -F /dev/ttyUSB0 115200 raw -echo -ixon -ixoff -crtscts cs8 -parenb -cstopb
perl tools/send_can_file.pl --file tools/test50.txt --count 10 \
  --wait 0.1 --device /dev/ttyUSB0
```

Remettre auparavant la carte à 115200. Observer les octets du fichier dans les messages `Broadcast` de source 10. Le script produit aussi `/tmp/test50.txt.concatenated_reference`. Malgré son nom, il écrit dans un **port série**, pas directement sur le CAN. Le découpage en plusieurs messages peut varier ; comparer le flux reconstitué. Le test automatique précédent fait cette comparaison plus simplement.

**Si anomalie :** vérifier TX/RX croisés, le port, le débit et l'absence d'un second logiciel ouvrant l'UART. Le firmware filtre les entrées CAN sur `protocol`, **pas sur `channel_id`** ; plusieurs tunnels de même protocole sur un même bus ne sont pas isolés par ce champ.

### T03 — Récepteur radio : `ROLE.sbus`

**Matériel :** pour l'essai automatique, un USB-UART TTL 3,3 V capable de **100000 bit/s, 8E2**. TX adaptateur → J4-4/RX et GND → J4-1 ; pas besoin de retour RX.

| Paramètre | Valeur pour le banc USB-UART |
| --- | --- |
| `ROLE.sbus` | `true` |
| `role.sbus.debug_uart_ttl` | `true` |
| `role.sbus.channel_mask` | `65535` : les 16 voies |
| `role.sbus.id` | `0` |

**Manipulation :**

1. Sélectionner **Futaba SBUS** dans `minican_tester`, choisir le port, préparer la session puis tester pendant 30 s.
2. Ouvrir **Panels → RC Panel** ou observer `dronecan.sensors.rc.RCInput` dans la GUI.
3. Vérifier `id=0`, 16 valeurs `rcin`, des valeurs entre 1000 et 2000, et les compteurs de concordance du tester.
4. Régler `role.sbus.channel_mask=15`, sauvegarder et redémarrer. Refaire le test : seules les quatre premières voies sont publiées.

**Réussite :** aucun message attendu manquant ni valeur incohérente pendant le test automatique ; `quality=255` sur les trames normales.

**Essai avec un vrai récepteur :** couper l'alimentation, remplacer l'USB-UART par un récepteur SBUS appairé à sa radio. Relier sortie SBUS → J4-4, GND → J4-1 et fournir l'alimentation prescrite pour le récepteur. Régler **`role.sbus.debug_uart_ttl=false`**, sauvegarder et redémarrer. Observer avec la GUI, sans lancer l'injection du tester. Les manches doivent faire varier les voies. Éteindre ensuite la radio : si le récepteur émet des trames de sécurité, le drapeau `FAILSAFE` doit suivre ; s'il cesse d'émettre, constater la disparition des messages. Rallumer et vérifier le retour.

**Si anomalie :** le réglage d'inversion est différent entre récepteur SBUS réel et USB-UART. `bus.serial.baudrate` ne choisit pas le débit SBUS. Une perte de trame signalée par le récepteur peut donner `quality=0`.

### T04 — GPS : `ROLE.gnss.ubx`

**Matériel :** récepteur GNSS compatible protocole **UBX**, antenne et accès à un ciel dégagé. GPS TX → J4-4/RX, GPS RX → J4-5/TX, GND → J4-1 ; alimentation selon le module. Vérifier les niveaux logiques.

| Paramètre | Valeur |
| --- | --- |
| `ROLE.gnss.ubx` | `true` |
| `bus.serial.baudrate` | `0` pour détection automatique |

Le firmware recherche les vitesses **57600, 115200 et 230400**. Un module réglé uniquement à 9600 doit être préparé par le tuteur ou utilisé à un débit fixe compatible ; la présence d'une sortie NMEA seule ne valide pas UBX.

**Manipulation :**

1. Observer `uavcan.equipment.gnss.Fix2` et `uavcan.equipment.gnss.Auxiliary` dans la GUI.
2. Dans `minican_tester`, choisir **GNSS UBX**, préparer la session et faire un essai de 30 s. Aucun USB-UART de PC n'est requis pour cet essai.
3. Laisser plusieurs minutes au GPS pour obtenir une position. Relever `status`, `sats_used`, `pdop`, latitude, longitude, altitude et vitesse.
4. Dans `Fix2`, `status=3` correspond à une position 3D ; latitude/longitude sont en **degrés × 10⁸**, les altitudes en millimètres et `ned_velocity` en m/s.
5. Au repos, vérifier une position proche du lieu du banc et une vitesse faible. Si possible, déplacer le montage de quelques mètres avec son alimentation et constater l'évolution de la position.

**Réussite :** réception répétée des messages de source 10 et position 3D cohérente en extérieur. Le `PASS` du tester exige seulement au moins un `Fix2` : **un `PASS` avec `NO_FIX` ne valide pas la réception satellite**. Si le ciel est inaccessible, noter « liaison OK, acquisition GNSS non validée ».

Enregistrer `gnss.log`. Pour un journal contenant uniquement la trajectoire réelle de la carte :

```sh
python3 ../TOOLS/uavcan_logger/log_to_kml.py resultats_tests/gnss.log \
  --output resultats_tests/gnss.kml --name "Essai MicroCAN"
```

Le convertisseur ne sépare pas les nœuds et ne vérifie pas la qualité du fix : ne pas mélanger ce journal avec des positions simulées ou d'autres GPS.

### T05 — Baromètre : `ROLE.i2c.barometer.mpl3115a2`

**Matériel :** module **MPL3115A2**, thermomètre et si possible baromètre de référence. SDA → J5-4, SCL → J5-5, GND → J5-1 ; alimentation compatible avec le module.

| Paramètre | Valeur |
| --- | --- |
| `ROLE.i2c.barometer.mpl3115a2` | `true` |
| `bus.i2c.frequency_khz` | `400` |
| `bus.i2c.pullup_resistor` | `true` pour le montage de référence |

Le capteur utilise l'adresse I2C `0x60`. Si le module comporte déjà des résistances de rappel, faire vérifier leur combinaison avec celles de la carte.

**Manipulation :**

1. Observer `uavcan.equipment.air_data.StaticPressure` et `StaticTemperature` pendant 30 s.
2. Comparer la température à l'ambiance, en tenant compte de l'échauffement du module. Approcher une main sans toucher les contacts : la température doit évoluer progressivement.
3. Comparer la pression à une référence mesurée à la même altitude. Une pression météo ramenée au niveau de la mer n'est pas directement comparable.
4. Enregistrer `barometre.log` avec les valeurs brutes, sans correction cachée dans le compte rendu.

**Critère attendu par DroneCAN :** `static_pressure` en **Pa** (environ 100000 Pa près du niveau de la mer) et `static_temperature` en **K** (environ 293 K à 20 °C), avec renouvellement régulier des mesures.

**Écart identifié dans le code actuel :** [baro_MPL3115A2_Role.cpp](COMMON/source/baro_MPL3115A2_Role.cpp) divise la pression brute par 6400, ce qui correspond à des hPa, et transmet la température sans ajouter 273,15. Si la GUI reçoit environ `1000` et `20`, noter **« acquisition présente, unités DroneCAN non conformes »**, pas « rôle OK ». Cette observation doit être confirmée au banc et remontée au tuteur. Le code ignore aussi les erreurs de lecture dans sa boucle de publication : des messages qui continuent n'assurent pas que les valeurs sont fraîches.

**Si aucun message :** vérifier référence du capteur, adresse, SDA/SCL, alimentation, résistances de rappel et retour en mode opérationnel.

### T06 — Magnétomètre : `ROLE.i2c.magnetometer.q5883`

**Matériel :** module **QMC5883/QMC5883L**, support non métallique. Ce n'est pas un HMC5883 interchangeable. SDA → J5-4, SCL → J5-5, GND → J5-1, alimentation adaptée.

| Paramètre | Valeur |
| --- | --- |
| `ROLE.i2c.magnetometer.q5883` | `true` |
| `bus.i2c.frequency_khz` | `400` |
| `bus.i2c.pullup_resistor` | `true` sur le montage de référence |
| `role.i2c.magnetometer.q5883.range` | `2` |
| `role.i2c.magnetometer.q5883.rot_deg` | `0` |
| `role.i2c.magnetometer.q5883.sensor_id` | `0` |

**Manipulation :**

1. Observer `uavcan.equipment.ahrs.MagneticFieldStrength2`, `sensor_id=0`, et les trois composantes du champ en gauss.
2. Éloigner moteur, aimants, outils et alimentation. Relever les trois valeurs, puis tourner doucement le module de 90° et 180°.
3. Vérifier que les composantes changent ; revenir à la position initiale et vérifier une valeur voisine.
4. Sans déplacer le module, essayer `rot_deg=90` après sauvegarde et redémarrage : le repère publié doit tourner, sans changement important de la norme du champ.

**Réussite :** mesures finies, répétées, sensibles à l'orientation ; ordre de grandeur du champ terrestre généralement autour de **0,25 à 0,65 gauss**, hors perturbations locales. Cela valide le fonctionnement, pas une calibration de boussole.

**Si anomalie :** vérifier le modèle réel et les perturbations métalliques. Un axe bloqué à une limite peut indiquer une saturation ; éloigner les aimants avant de changer la gamme.

### T07 — Microphone : `ROLE.adc.microphone.im68a130`

**Matériel :** montage microphone analogique **IM68A130(A)** adapté, haut-parleur et générateur de son sinusoïdal. La sortie analogique préparée par le tuteur va sur **J6-4/PA04**, masse sur J6-1. Respecter l'alimentation propre au microphone et son conditionnement : le signal d'entrée doit rester dans la plage **0–3,3 V**. Ne pas connecter directement une sortie haut-parleur à l'ADC.

| Paramètre | Valeur |
| --- | --- |
| `ROLE.adc.microphone.im68a130` | `true` |
| `role.adc.microphone.period_ms` | `200` |

**Manipulation :**

1. Vérifier que la GUI a chargé les DSDL custom au lancement (§2.4).
2. Dans sa console, après `cible = 10`, coller :

```python
subscribe(dronecan.thirdparty.microcan.audio.Spectrum, lambda e: print_yaml(e.message) if e.transfer.source_node_id == cible else None)
```

3. Observer pendant 10 s dans le calme, puis jouer un son de **1000 Hz** à volume modéré pendant 10 s.
4. Rechercher parmi `bins` une composante forte proche de **984 Hz** ou **1031 Hz**. Le calcul découpe les fréquences par pas d'environ 47 Hz : il n'affiche pas nécessairement 1000 exactement.
5. Essayer ensuite 1500 Hz, puis augmenter légèrement le volume sans saturer. La composante doit se déplacer et son `level` augmenter.
6. Enregistrer `microphone.log` dans chaque situation.

**Réussite :** messages `microcan.audio.Spectrum`, ID **20900**, environ 5 fois/s ; `adc_bits=13`, `status & 1` non nul, au maximum 10 composantes et réaction aux deux sons. `status & 2` signale un écrêtage ; réduire le niveau. `status & 4` signale une interruption/reprise d'acquisition, à relever si elle se répète.

`level` va de 0 à 65535 ; pour cette configuration, **dBFS = 72,2472 × (level / 65535 − 1)**. Il ne s'agit pas de décibels acoustiques calibrés. Une liste vide avec `VALID` peut être normale dans le silence. **`VALID` prouve que l'ADC acquiert, pas qu'un microphone est réellement connecté** : la réponse au son est indispensable.

**Si anomalie :** vérifier alimentation et sortie analogique au multimètre/oscilloscope, namespace DSDL et conflit avec PWM CH6. Le rôle a des tests logiciels, mais sa documentation indique qu'une validation matérielle reste à réaliser.

### T08 — Lumière RGBW : `ROLE.i2c.light.opt4060`

**Matériel :** module **OPT4060**, lampe stable, carton opaque et éventuellement éclairage rouge/vert/bleu. SDA → J5-4, SCL → J5-5, GND → J5-1 ; alimentation adaptée.

| Paramètre | Valeur du premier essai |
| --- | --- |
| `ROLE.i2c.light.opt4060` | `true` |
| `bus.i2c.frequency_khz` | `400` |
| `bus.i2c.pullup_resistor` | `true` sur le montage de référence |
| `role.i2c.light.opt4060.address` | `68` = `0x44`, à adapter aux straps du module |
| `role.i2c.light.opt4060.publish_hz` | `10` |
| `role.i2c.light.opt4060.scan_hz` | `0` |
| `role.i2c.light.opt4060.sensor_id` | `0` |
| `role.i2c.light.opt4060.use_interrupt` | `false`, sans fil INT |

**Manipulation, mode périodique :**

1. Observer `microcan.light.Measurement`, ID **20901**. Dans la console GUI :

```python
subscribe(dronecan.thirdparty.microcan.light.Measurement, lambda e: print_yaml(e.message) if e.transfer.source_node_id == cible else None)
```

2. Relever `rgbw` à lumière stable, sous un carton opaque, puis sous une lampe.
3. Vérifier l'ordre **rouge, vert, bleu, clair** des quatre valeurs. « W » désigne ici le canal large bande, pas une LED blanche.
4. Vérifier `status=1` (valide), `reason=0` (périodique), un horodatage `timestamp_us` croissant et une cadence voisine de 10 Hz.

**Manipulation, mode sur changement :** régler `scan_hz=50`, `publish_hz=10`, `delta_rel_pct=5`, `delta_abs=1024`, `heartbeat_ms=1000`, puis sauvegarder et redémarrer.

1. Maintenir un éclairage stable : après le premier état, des messages de maintien (`reason=2`) sont attendus environ chaque seconde s'il n'y a pas de changement détecté.
2. Couvrir puis découvrir le capteur : observer des messages de changement (`reason=1`) ou d'état (`reason=3`), avec les valeurs correspondantes.
3. Les acquisitions sont faites à 50 Hz mais les publications sont plafonnées à 10 Hz. Un éclairage secteur qui clignote peut produire des événements même lorsque la lampe semble stable.
4. Pour tester l'interruption, couper le courant, ajouter **INT → J7-4/PA08**, régler `use_interrupt=true`, redémarrer et refaire l'essai périodique.

**Réussite :** quatre canaux qui réagissent, cadence et motifs conformes au mode, retour à des mesures valides après les changements. Ces valeurs sont des **codes de capteur**, pas des lux ni des couleurs d'écran. `status=2` indique une saturation et `status=4` une erreur ; dans ce dernier cas, quatre zéros ne signifient pas l'obscurité.

**Si anomalie :** vérifier adresse `0x44` à `0x47`, fil INT et DSDL custom. Ne pas chercher les anciens messages `opt.*` dans `KeyValue`. À 400 kHz, `0 < scan_hz < publish_hz` est une configuration invalide, pas un mode de fonctionnement.

### T09 — Distance : `ROLE.i2c.range.vl53l4cx`

**Matériel :** module **VL53L4CX** compatible, règle/mètre et plaque mate claire. SDA → J5-4, SCL → J5-5, GND → J5-1 ; alimentation adaptée. Si le module possède XSHUT, le maintenir dans l'état actif prescrit par sa documentation ; le rôle ne lui réserve pas de broche dédiée.

| Paramètre | Valeur |
| --- | --- |
| `ROLE.i2c.range.vl53l4cx` | `true` |
| `bus.i2c.frequency_khz` | `400` |
| `bus.i2c.pullup_resistor` | `true` sur le montage de référence |
| `role.i2c.range.vl53l4cx.period_ms` | `200` |
| `role.i2c.range.vl53l4cx.sensor_id` | `0` |

**Manipulation :**

1. Observer `uavcan.equipment.range_sensor.Measurement` de source 10.
2. Placer la cible en face à **0,20 m**, puis **0,50 m**, puis **1,00 m**, mesurés depuis la face du capteur.
3. À chaque position, attendre la stabilisation et relever dix mesures de `range`, avec `reading_type`.
4. Enlever la cible ou viser une zone sans retour exploitable : une lecture non valide doit être signalée comme telle, puis les mesures doivent revenir quand la cible est replacée.

**Réussite :** environ 5 messages/s, `sensor_id=0`, distance en **mètres**, ordre et valeurs cohérents avec la règle. Pour ce contrôle simple, proposer **±5 cm** sur la cible mate entre 0,20 et 1 m ; ce n'est pas une qualification de précision constructeur. Une lecture valide a le type `READING_TYPE_VALID_RANGE` ; sans cible valide, le firmware publie `READING_TYPE_UNDEFINED` et `NaN`.

**Si anomalie :** vérifier référence exacte du module, adresse `0x29`, film de protection éventuel, XSHUT et surface visée. Ne pas prendre `timestamp=0` pour une panne : ce rôle n'utilise pas une horloge réseau synchronisée.

### T10 — Bande de LED : `ROLE.led2812`

**Matériel :** bande de **8 WS2812**, alimentation 5 V adaptée et liaison de données compatible. **DIN → J5-4/PB07**, GND commun. Si la bande 5 V ne garantit pas la reconnaissance d'un signal 3,3 V, utiliser le tampon de niveau prévu par le tuteur, par exemple un 74AHCT125 alimenté en 5 V. Ne pas relier DIN à DOUT.

**Paramètres :** `ROLE.led2812=true`, `role.led2812.led_number=8`. Aucun capteur I2C ne reste branché sur SDA pour ce test.

Dans la **console GUI**, envoyer les huit LED en rouge à faible luminosité :

```python
ind = dronecan.uavcan.equipment.indication
broadcast(ind.LightsCommand(commands=[ind.SingleLightCommand(light_id=i, color=ind.RGB565(red=4, green=0, blue=0)) for i in range(8)]))
```

Puis vert et bleu, une commande à la fois :

```python
broadcast(ind.LightsCommand(commands=[ind.SingleLightCommand(light_id=i, color=ind.RGB565(red=0, green=8, blue=0)) for i in range(8)]))
broadcast(ind.LightsCommand(commands=[ind.SingleLightCommand(light_id=i, color=ind.RGB565(red=0, green=0, blue=4)) for i in range(8)]))
```

Éteindre tout, puis allumer seulement la première LED :

```python
broadcast(ind.LightsCommand(commands=[ind.SingleLightCommand(light_id=i, color=ind.RGB565(red=0, green=0, blue=0)) for i in range(8)]))
broadcast(ind.LightsCommand(commands=[ind.SingleLightCommand(light_id=0, color=ind.RGB565(red=4, green=0, blue=0))]))
```

Essayer ensuite `light_id=7` pour la dernière LED, puis réenvoyer la commande qui éteint tout.

**Réussite :** huit LED dans la bonne couleur, commande individuelle de la première et de la dernière, extinction complète. Les indices sont **0 à 7**. Le format RGB565 limite rouge/bleu à 31 et vert à 63. Ce rôle ne renvoie pas d'accusé de réception : photographier le résultat physique.

**Si anomalie :** vérifier DIN, GND, niveau logique et alimentation. Ce rôle pilote une bande **externe**, pas la LED interne d'identification. Il partage PB07 et TIM3 avec le voltmètre et certaines sorties PWM.

### T11 — Jauge de batterie : `ROLE.voltmeter`

**Matériel :** même bande de 8 WS2812 que T10, alimentation réglable sur VBUS et multimètre. Utiliser l'alimentation de laboratoire pour simuler la batterie ; ne pas décharger une vraie batterie pour atteindre les seuils.

| Paramètre | Valeur |
| --- | --- |
| `ROLE.voltmeter` | `true` |
| `role.voltmeter.cells` | `3` : simulation d'une batterie 3 cellules |
| `role.voltmeter.brightness` | `0.2` |
| `role.voltmeter.gps_speed_off_mps` | `0` pour commencer |

**Manipulation, tension :**

1. Régler successivement VBUS à **12,7 V**, **11,55 V**, puis **10,1 V**. Vérifier chaque tension au multimètre ; attendre la stabilisation de la mesure.
2. Attendre respectivement huit LED vertes, une jauge intermédiaire autour de la moitié, puis huit LED rouges clignotantes. De petites différences de mesure peuvent modifier d'une LED le point intermédiaire.
3. Revenir à 12 V. Le rôle utilise sa propre lecture ADC ; `ROLE.health.survey` peut rester désactivé.

**Manipulation, extinction avec vitesse GPS simulée :**

1. Régler `role.voltmeter.gps_speed_off_mps=3`, sauvegarder et redémarrer. Laisser le rôle GNSS désactivé.
2. Dans la console GUI, envoyer une vitesse de 5 m/s pendant 5 s :

```python
fix = dronecan.uavcan.equipment.gnss.Fix2(status=3, ned_velocity=[5.0, 0.0, 0.0])
broadcast(fix, interval=0.2, duration=5)
```

3. Les LED doivent s'éteindre pendant les messages, puis se rallumer environ **2 s après le dernier message**.
4. Refaire l'envoi rapide, puis tester la réapparition à faible vitesse :

```python
stop()
broadcast(dronecan.uavcan.equipment.gnss.Fix2(status=3, ned_velocity=[2.0, 0.0, 0.0]), interval=0.2, duration=5)
```

Le seuil d'extinction est strictement supérieur à 3 m/s ; le retour avec des données valides se fait sous **2,5 m/s**, pour éviter le clignotement près du seuil.

**Réussite :** affichage cohérent aux trois tensions, extinction à 5 m/s, retour à 2 m/s et après disparition des messages. Photographier les états. Ces messages simulés testent la jauge, **pas un récepteur GPS**.

**Complément avec `uavcan_replay` :** enregistrer les messages du test GPS simulé dans `resultats_tests/voltmeter_gps.log` avec le logger. Arrêter le logger et fermer la GUI, afin de libérer son identifiant source 126. Rejouer uniquement ces positions :

```sh
../TOOLS/uavcan_replay/minican_replay --iface can0 \
  --dsdl "$HOME/DEV/STM32/UAVCAN/DSDL" \
  --log resultats_tests/voltmeter_gps.log \
  --filter 126:uavcan.equipment.gnss.Fix2
```

Le programme s'appelle **`minican_replay`**, même si son dossier s'appelle `uavcan_replay`. Il conserve les numéros sources du journal. Observer à nouveau les LED ; après le dernier message rapide, elles doivent revenir au plus tard après expiration des données. Ne pas rejouer un journal complet contenant des commandes moteurs/servos ou les `NodeStatus` d'une carte encore connectée.

### T12 — Servos PWM : `ROLE.servo.pwm`

**Matériel :** oscilloscope et, après vérification du signal, servo PWM classique adapté. Signal servo → **J7-4/CH1**, GND commun ; alimentation servo séparée à sa tension nominale.

| Paramètre | Valeur |
| --- | --- |
| `ROLE.servo.pwm` | `true` |
| `role.servo.pwm.map_index1` | `0` |
| `role.servo.pwm.channel_mask` | `1` : CH1 seul |
| `role.servo.pwm.ch1-4.frequency` | `50` |
| `role.servo.pwm.ch1-4.shot125` | `false` |
| `role.servo.pwm.ch5-7.frequency` | `50` |
| `role.servo.pwm.ch5-7.shot125` | `false` |

**Manipulation :**

1. Commencer sans servo : pointe de sonde sur CH1, masse de sonde sur GND.
2. Dans `minican_tester`, choisir **PWM Servo**, préparer la session. Le curseur CH1 doit correspondre à l'actuateur 0.
3. Commander successivement `0`, `-0.5`, `+0.5`. Mesurer les impulsions attendues : **1500, 1250 et 1750 µs**, répétées toutes les **20 ms**.
4. Couper, connecter le servo dégagé de toute mécanique, remettre sous tension et reprendre les petites excursions. Revenir au centre.
5. Pour tester toute la course à l'oscilloscope, débrancher le servo hors tension puis lancer **Start Long Test** : il balaie **−1 à +1**, soit **1000 à 2000 µs**. Un servo ne doit subir ce balayage que si ses limites le permettent.
6. Tester ensuite les sept canaux avec `channel_mask=127` et le tableau suivant. Aucun périphérique UART, I2C, microphone ou bande LED ne reste connecté à leurs broches réutilisées.

| Canal | Signal | Masque pour ce canal seul |
| --- | --- | --- |
| CH1 | J7-4 / PA08 | `1` |
| CH2 | J7-5 / PA09 | `2` |
| CH3 | J7-6 / PA10 | `4` |
| CH4 | J7-7 / PA11 | `8` |
| CH5 | J4-4 / PB04 | `16` |
| CH6 | J6-4 / PA04 | `32` |
| CH7 | J5-4 / PB07 | `64` |

Les identifiants de commande sont attribués dans l'ordre des **canaux actifs** à partir de `map_index1`. Avec le masque 127, CH1 à CH7 utilisent les IDs 0 à 6. Avec CH7 seul et `map_index1=0`, son ID est **0**, pas 6.

**Variante OneShot125, oscilloscope uniquement :** régler la fréquence du groupe testé à **400 Hz** et son `shot125=true`, sauvegarder et redémarrer. Le centre doit passer à **187,5 µs**, les extrêmes à **125 et 250 µs**, avec une période de **2,5 ms**. Tester les deux groupes TIM1/CH1–4 et TIM3/CH5–7. Rétablir 50 Hz et `shot125=false` avant de rebrancher un servo classique.

**Réussite :** chaque broche produit la période et les largeurs attendues, les autres canaux ne suivent pas une commande qui ne leur est pas destinée, et le servo suit les petites consignes. Pour ce TP, relever les écarts à l'oscilloscope et viser moins de 2 %.

Le `PASS` du tester confirme seulement l'émission CAN sans erreur locale : **aucun retour matériel PWM n'est lu**. Le firmware conserve la dernière consigne ; arrêter le tester ne remet pas automatiquement les servos au centre.

### T13 — Servo intelligent : `ROLE.servo.smart`

**Matériel :** servo **STS3032** compatible avec le pilote, alimentation nominale et ID connu. Utiliser un seul servo, mécaniquement libre. Son bus DATA à un fil va sur **J4-5/TX/PB03**, avec GND commun : le pilote travaille en UART **half-duplex**. Utiliser l'interface électrique adaptée au servo ; ne pas improviser un pont TX–RX comme pour un UART à deux fils.

Exemple pour un servo dont l'ID interne est **1** :

| Paramètre | Valeur |
| --- | --- |
| `ROLE.servo.smart` | `true` |
| `role.servo.smart.map_index1` | `1` |
| `role.servo.smart.num_servos` | `1` |
| `role.servo.smart.status_frequency` | `10` |

`map_index1` doit correspondre à l'ID du servo, pas au numéro de la carte CAN. Le firmware détecte le débit parmi **1 000 000, 500 000 et 250 000 bit/s** ; `bus.serial.baudrate` ne règle pas ce rôle. **Au démarrage, le firmware commande déjà le centre.**

**Manipulation :**

1. Alimenter le servo avant ou en même temps que la carte ; laisser le scan de démarrage se terminer.
2. Dans `minican_tester`, choisir **Smart Servo**, préparer la session et sélectionner **Actuator ID = 1**.
3. Commander `0`, `-0.1`, `+0.1`, puis `0`. Vérifier mouvement réel et évolution du retour.
4. Observer `uavcan.equipment.actuator.Status` dans la GUI : `actuator_id=1`, `position`, `speed`, `power_rating_pct`.
5. Seulement sur un servo dont toute la course est libre, lancer l'essai long de 30 s. Il balaie **−1 à +1**, donc toute la plage de position du pilote.
6. Revenir au centre avant de couper la puissance.

**Réussite fonctionnelle :** mouvement cohérent, retour répété du bon servo et absence de pertes persistantes. La charge doit rester raisonnable sans effort mécanique ; une forte charge à l'arrêt peut signaler une butée.

**Limites à relever :** le `PASS` automatique exige seulement un retour d'état. Le firmware copie actuellement une position normalisée **−1 à +1** et une vitesse brute du STS3032 dans `Status`, alors que le DSDL standard attend des unités physiques (radians et rad/s pour un servo rotatif). Ne pas valider une mesure angulaire à partir des graduations « degrés » du tester ; consigner cet écart d'unités. `force=NaN` est normal ; le message actuel ne transmet pas le courant, la tension ou la température du servo. Le réglage `status_frequency=0` supprime le retour périodique, mais les commandes traitées peuvent encore déclencher un retour immédiat.

**Si anomalie :** vérifier modèle, ID réel, alimentation de puissance, DATA half-duplex et débit. Un servo qui manque dans la plage d'IDs configurée peut empêcher le rôle de démarrer.

### T14 — Contrôleur moteur : `ROLE.esc.dshot`

**Matériel :** ESC compatible **DShot300 bidirectionnel**, moteur sans hélice fixé au banc, alimentation de puissance et coupure accessible. Signal ESC → **J7-4/CH1**, GND ESC → GND carte. Alimenter l'ESC selon sa fiche technique. La configuration actuelle utilise DShot300 avec télémétrie bidirectionnelle et EDT ; la télémétrie revient sur le fil DShot. Le fil UART de télémétrie de certains ESC n'est pas utilisé ici.

| Paramètre | Valeur |
| --- | --- |
| `ROLE.esc.dshot` | `true` |
| `role.esc.dshot.map_index1` | `0` |
| `role.esc.dshot.channel_mask` | `1` : CH1 seul |
| `role.esc.dshot.cmd_rate` | `100`, conserver la valeur du firmware |
| `role.esc.dshot.rpm_freq_div` | `10` pour publier la télémétrie |
| `role.esc.dshot.motor_poles` | Nombre **réel et pair** de pôles du moteur, par exemple `14` seulement pour un moteur à 14 pôles |

**Attention au réglage de cadence :** dans le firmware, `cmd_rate` est une **période en ticks ChibiOS**, pas une fréquence en Hz. L'étiquette `Cmd rate Hz` du tester décrit sa propre cadence d'envoi CAN et ne doit pas servir à interpréter la période du firmware.

**Manipulation :**

1. Vérifier l'absence d'hélice et de pièce mobile accessible. Démarrer avec consigne zéro.
2. Choisir **DSHOT ESC** dans `minican_tester`, préparer la session, sélectionner l'index télémétrique **0**.
3. Augmenter progressivement la commande manuelle de 0 à 200, puis 400 si nécessaire et si le montage le permet. Ce sont des valeurs `RawCommand`, pas un pourcentage ; le firmware les divise par quatre pour former la consigne DShot.
4. Vérifier le démarrage du moteur, puis l'augmentation du régime. Revenir à zéro et constater l'arrêt.
5. Observer `uavcan.equipment.esc.Status` de source 10 : `esc_index=0`, régime `rpm`, tension en V, courant en A, température en K si ces mesures sont disponibles. Pour afficher des °C, soustraire 273,15.
6. Si le banc autorise le scénario, lancer **Start Long Test** pendant 30 s : le tester balaie **0 à 1200**, puis revient. Ce scénario n'est pas limité par le dernier réglage manuel du curseur.
7. Répéter pour CH2, CH3 et CH4, en recâblant hors tension et avec les masques 2, 4 et 8. Avec un seul canal actif et `map_index1=0`, l'index reste 0.

**Arrêt obligatoire, après un test normal ou interrompu :**

1. Cliquer **Stop Test** s'il tourne encore, puis déconnecter le CAN du tester ou fermer le tester pour qu'il cesse d'émettre.
2. Dans la console GUI, arrêter ses autres émissions puis envoyer explicitement zéro sur les 20 indices :

```python
stop()
broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0] * 20), interval=0.02, duration=1)
```

3. Vérifier l'arrêt physique, puis couper l'alimentation de puissance. Si le moteur ne s'arrête pas, utiliser immédiatement la coupure de puissance.

**Le code du rôle conserve la dernière consigne et ne prévoit pas de délai d'arrêt sur perte des commandes CAN. `Stop Test`, `Ctrl+C` ou fermer la GUI ne suffisent donc pas.**

**Réussite :** démarrage/arrêt contrôlés, régime cohérent et télémétrie du bon index sur chaque sortie testée. Si un tachymètre est disponible, comparer le régime : un mauvais nombre de pôles fausse directement le résultat. Une mesure EDT absente ou vieille d'au moins 3 s devient `NaN` ; cela ne signifie pas nécessairement que la commande moteur est en panne.

Le `PASS` automatique repose sur la réception d'au moins un `ESC.Status`, sans prouver la précision des mesures. Les compteurs « ack/RTT » rapprochent temporellement commande et télémétrie ; ce protocole ne fournit pas un acquittement individuel des commandes. Avec `rpm_freq_div=0`, les publications CAN sont désactivées : l'absence d'état est alors attendue.

**Complément EDT :** si l'ESC fournit stress/états EDT, observer `dronecan.protocol.FlexDebug`, `id=2000+esc_index`. L'absence de ces messages est normale pour un ESC ne fournissant pas ces champs. Leur décodage est décrit dans [dshot_telemetry.md](docs/software/roles/dshot_telemetry.md). Ne pas bloquer ou surcharger un moteur pour forcer une alarme.

### T15 — Modèle de développement : `ROLE.template`

**Matériel :** PC-CAN, carte et adaptateur USB-UART sur la **console J3** (§2.3).

**Préparation par le tuteur :** `USE_TEMPLATE_ROLE` vaut actuellement `false` dans [roleConf.h](COMMON/source/roleConf.h). Pour cet essai, passer cette définition à `true`, conserver **`TRACE`**, reconstruire et charger le firmware selon [build_flash.md](docs/software/build_flash.md). Une simple activation dans la GUI n'est pas possible si le rôle n'est pas compilé. Sans ce firmware, noter **« non testé : rôle exclu de la compilation »**.

| Paramètre | Valeur |
| --- | --- |
| `ROLE.template` | `true` |
| `role.template.log_every` | `1` |

**Manipulation :**

1. Laisser la GUI connectée avec son Node ID 126. Elle émet son `uavcan.protocol.NodeStatus`.
2. Vérifier ces `NodeStatus` dans Bus Monitor et regarder en parallèle le terminal série **J3 à 115200 bit/s**.
3. Dans le terminal série, vérifier un texte du type `UAVDbg : templateRole: node=126 uptime=...`.
4. Passer `log_every=5`, sauvegarder et redémarrer : un log doit être produit toutes les cinq réceptions de `NodeStatus`.
5. Passer `log_every=0`, sauvegarder et redémarrer : ces logs doivent cesser, tout en laissant le nœud fonctionner.

**Réussite :** réaction aux messages d'état, division de la cadence par cinq et suppression des logs à zéro. S'il y a plusieurs nœuds PC actifs, tous leurs messages d'état peuvent être comptés ; fermer les outils inutiles pour mesurer le rapport.

Dans cette application, `node.infoCb` aboutit à une trace série lorsque `TRACE` est activé. **Le modèle ne publie pas ces textes en `uavcan.protocol.debug.LogMessage` sur le CAN.** Conserver une capture du terminal série avec le journal CAN des `NodeStatus` utilisés comme stimulus. Sans `TRACE`, l'absence de texte ne permet pas de conclure sur la réception.

Après le test, désactiver le rôle. Le tuteur remet la définition de compilation et le firmware de référence si ce changement n'est pas destiné à être conservé.

### T16 — Identification : `ROLE.identification`

**Matériel :** aucun périphérique supplémentaire ; PC-CAN et carte avec le **firmware corrigé**. La console J3 reste disponible si `ROLE.shell=true`, mais elle n’est pas nécessaire pour sortir du mode.

**Configuration de départ :** conserver une configuration déjà validée avec au moins un rôle actif autre que le shell. Sur un banc sans périphérique, utiliser celle de T01 (`ROLE.health.survey=true`). Ne pas désactiver ce rôle pour entrer en identification. Faire **Fetch All → Save to File** et conserver `resultats_tests/parametres_avant_identification.json`.

**Manipulation :**

1. Arrêter tous les essais et couper les alimentations moteurs/servos.
2. Régler `ROLE.identification=true`, sauvegarder et redémarrer.
3. Observer le motif violet sur la LED **interne**. La carte doit revenir dans la GUI après son redémarrage, publier `NodeStatus` en mode **MAINTENANCE** et conserver ce motif même une fois son Node ID attribué.
4. Cliquer **Fetch All** : tous les paramètres des rôles doivent rester visibles. Comparer leurs indicateurs `ROLE.*` et réglages `role.*` à la sauvegarde : ils doivent être identiques, à l'exception de `ROLE.identification`. Le rôle actif avant le test doit toujours afficher `true`, mais ne plus fonctionner. Modifier temporairement `hardware.nickname`, sauvegarder et relire la valeur. Conserver le nom original pour le rétablir à la fin.
5. Depuis la GUI, remettre uniquement `ROLE.identification=false`, cliquer **Store All** puis **Restart**. Vérifier le retour en mode **OPERATIONAL**, le motif affichant le Node ID et la reprise du rôle précédemment actif avec ses réglages initiaux. Faire **Fetch All** pour confirmer leur conservation. Aucune réactivation individuelle des rôles ni commande série ne doit être nécessaire.
6. Vérifier aussi un démarrage avec ID dynamique : sauvegarder `uavcan.node_id=0` et `ROLE.identification=true`, lancer le serveur d'allocation de la GUI et redémarrer. La carte doit obtenir un ID et rester configurable. Utiliser cet ID pour la suite, puis rétablir 10 après l'essai.

**Vérification de la mise à jour, avec le tuteur :**

1. Garder `ROLE.identification=true` et la carte en mode MAINTENANCE. Utiliser une image compatible qui contient ce correctif, par exemple `microcan/build_MICROCAN/LAST_MICROCAN.uavcan.bin` issue de la compilation corrigée. Ne pas choisir le `.bin` brut sans en-tête UAVCAN.
2. Dans **Node Properties**, cliquer **Update Firmware**, choisir cette image et suivre le transfert jusqu'au redémarrage. Le mode peut passer temporairement à `SOFTWARE_UPDATE`.
3. Vérifier que la carte réapparaît après le flash, consulter sa version et refaire **Fetch All**. Si identification est toujours activé, le mode revient à MAINTENANCE avec la LED violette.
4. Rétablir le surnom, le Node ID prévu et `ROLE.identification=false`, sauvegarder puis redémarrer.

**Réussite :** LED d'identification et services CAN fonctionnent ensemble ; les autres rôles, sauf le shell activé, sont suspendus, leurs paramètres restent visibles et inchangés, puis ils reprennent après désactivation et redémarrage. Lecture/écriture des paramètres, allocation dynamique et mise à jour complète du firmware réussissent. Sans flash effectivement réalisé, noter « configuration OK, mise à jour non testée ».

Le shell reste disponible en identification uniquement si `ROLE.shell=true` (voir T17). Les autres rôles, y compris HealthSurvey, restent arrêtés, même si leurs paramètres sont activés. Pour vérifier cette isolation sans actionneur, laisser par exemple `ROLE.health.survey=true` avant l'entrée dans ce mode : `NodeStatus` doit continuer, tandis que les publications de température/tensions cessent ; elles reprennent après sortie du mode et redémarrage.

### T17 — Shell de diagnostic : `ROLE.shell`

**But :** rendre la console série disponible sur demande, en réservant sa mémoire seulement lorsqu’elle est active. UAVCAN sert à activer ce rôle ; les commandes du shell passent ensuite sur J3, pas sur le CAN.

**Matériel :** banc CAN commun, adaptateur USB-UART TTL **3,3 V**, trois fils. Couper l’alimentation, relier TX de l’adaptateur à J3 broche 7 (DBG_RX), RX à J3 broche 6 (DBG_TX) et GND à J3 broche 1. Ne pas connecter le fil d’alimentation de l’adaptateur. J4 reste disponible pour ses autres rôles.

1. Dans DroneCAN GUI, faire **Fetch All**. Si `ROLE.shell` manque, faire vérifier la compilation par le tuteur : `NOSHELL=1` exclut le rôle.
2. Désactiver les autres rôles comme au §2.5. Régler `ROLE.identification=false` et `ROLE.shell=false`, **Store All**, **Restart**. Le nœud doit répondre par CAN, mais aucune commande saisie sur J3 ne doit recevoir de réponse.
3. Régler uniquement `ROLE.shell=true`, **Store All**, **Restart**. Ouvrir le terminal série à **115200 bit/s, 8N1, sans contrôle de flux**, puis appuyer sur Entrée. Le prompt doit apparaître. `bus.serial.baudrate` ne modifie pas ce débit de diagnostic.
4. Saisir successivement `info`, `mem`, `threads`, `adc`, `st ROLE.shell`, `can`. Vérifier les réponses : version et Node ID cohérents avec la GUI, mémoire disponible, threads `Enhanced_shell` et `serialPrint`, mesure ADC plausible. Appuyer sur la flèche vers le haut pour rappeler une commande, puis essayer la touche Tab pour la compléter. `threads` nécessite les statistiques de débogage, actives dans la compilation habituelle.
5. Laisser `ROLE.shell=true`. Activer `ROLE.identification=true` dans la GUI, sauvegarder et redémarrer : LED violette, CAN accessible et **shell toujours disponible**. Vérifier `can`, `st ROLE.identification` et `st ROLE.shell` dans le terminal. Relire les paramètres : `ROLE.shell` doit toujours être `true`.
6. En restant en identification, régler `ROLE.shell=false` dans la GUI, sauvegarder et redémarrer : le shell doit devenir silencieux, tandis que la LED violette et les services CAN restent actifs. Réactiver `ROLE.shell`, sauvegarder et redémarrer : il doit revenir sans sortir de l’identification.
7. Désactiver seulement l’identification, sauvegarder et redémarrer : le shell doit rester disponible. Désactiver ensuite `ROLE.shell`, sauvegarder et redémarrer : il doit redevenir silencieux ; paramètres, redémarrage et mise à jour CAN restent accessibles.

**Contrôle mémoire avec le tuteur :** le silence série ne prouve pas à lui seul que la mémoire est libérée. Comparer deux démarrages avec le même firmware et les mêmes autres paramètres, shell désactivé puis activé. Au débogueur SWD, arrêter la carte après démarrage et relever la mémoire libre ainsi que les threads. Shell désactivé : aucun contexte de console alloué, aucun thread `Enhanced_shell` ni `serialPrint`. Shell activé : un contexte et deux piles alloués. Les petits objets fixes du pilote UART restent présents dans les deux cas. Les tailles et la méthode de contrôle du binaire sont détaillées dans [shell.md](docs/software/roles/shell.md).

**Réussite :** activation et désactivation se comportent comme indiqué dans les deux modes ; commandes, historique et autocomplétion fonctionnent. Consigner séparément le contrôle mémoire s’il n’a pas été effectué. La désactivation prend effet au redémarrage, sans arrêt à chaud.

## 4. Contrôles complémentaires après les essais individuels

Ces contrôles ne remplacent aucune fiche individuelle. Les faire une fois chaque périphérique validé séparément.

### Partage de l'I2C et coexistence des capteurs

1. Hors tension, brancher OPT4060 et VL53L4CX sur le même SDA/SCL, avec alimentations et résistances de rappel adaptées. Leurs adresses sont différentes.
2. Activer leurs deux rôles, avec I2C à 400 kHz, OPT4060 à 10 Hz sans interruption, VL53L4CX à 200 ms.
3. Ajouter le microphone sur PA04 et activer son rôle à 200 ms.
4. Vérifier pendant 2 min que les trois messages continuent, que lumière, son et distance réagissent chacun à leur stimulus et que `uptime` ne revient pas à zéro.
5. Ajouter éventuellement `ROLE.health.survey=true` pour contrôler que température et alimentation restent publiées.

**Résultat :** aucun redémarrage, mesures fraîches et valides. La lumière infrarouge du télémètre peut influencer OPT4060 : les acquisitions ne sont pas coordonnées pour l'éliminer.

Pour tester une panne de capteur en cours de fonctionnement, utiliser seulement un montage de coupure prévu par le tuteur, qui évite d'alimenter un module éteint par SDA/SCL. Ne pas arracher au hasard un fil sous tension. OPT4060 doit publier une erreur, VL53L4CX une mesure indéfinie ; après trois erreurs consécutives, ces deux pilotes tentent une réinitialisation toutes les 5 s. Relever le délai réel de reprise après rétablissement. Une absence dès le démarrage est un autre cas : elle peut empêcher le rôle de démarrer.

### Comprendre les conflits

| Ressource | Fonctions qui se gênent |
| --- | --- |
| UART J4 / USART2 | GNSS, SBUS, tunnel et servo intelligent ; ne pas les activer ensemble. |
| PB04 / J4-4 | PWM CH5 et réception UART. |
| PA04 / J6-4 | PWM CH6 et microphone. |
| PA08 / J7-4 | PWM CH1, DShot CH1 et interruption OPT4060. |
| PB07 / J5-4 | I2C, PWM CH7, bande LED et voltmètre. |
| TIM1 | PWM CH1–4 et DShot, même si des broches différentes sont choisies. |
| TIM3 | PWM CH5–7, bande LED et voltmètre. |

Il n'est donc pas possible de valider tous les rôles en les activant simultanément. En cas de conflit, revenir à un seul rôle et redémarrer ; utiliser la console J3 seulement si `ROLE.shell` était déjà activé et a pu démarrer. Si CAN et shell sont tous deux indisponibles, le tuteur doit utiliser la programmation filaire.

## 5. Autres usages des outils de `TOOLS`

### Sauvegarder et modifier les paramètres en fichier texte

La GUI suffit pour les fiches précédentes. L'outil `uavcan_param_editor` offre une alternative reproductible :

```sh
../TOOLS/uavcan_param_editor/uavcan_param_editor --iface can0 --node-id 123 \
  --file resultats_tests/parametres_banc.txt --fetch
```

Le fichier contient une section `[node]` et une section `[param]` par nœud. Garder une copie originale. Modifier les valeurs de la carte 10 uniquement, puis ajouter au début du fichier :

```text
@cmd: save, store, reboot
```

Réexécuter **sans `--fetch`** :

```sh
../TOOLS/uavcan_param_editor/uavcan_param_editor --iface can0 --node-id 123 \
  --file resultats_tests/parametres_banc.txt
```

Lire les réponses, puis vérifier les paramètres après redémarrage dans la GUI. `save` applique les valeurs, `store` les rend persistantes et `reboot` redémarre. Ne pas reprendre tel quel le fichier d'exemple `../TOOLS/uavcan_param_editor/uavcan_params.txt` : il contient plusieurs cartes et des paramètres d'anciennes versions. Éviter `--watch` pour un premier TP, car une sauvegarde de fichier peut alors agir immédiatement sur la carte.

### Firmware et définitions des messages

`uavcan_fw_updater` met à jour le firmware ; il ne teste pas les capteurs. Le tuteur choisit l'image compatible et suit son [README](../TOOLS/uavcan_fw_updater/README.md) ou la [procédure de flash du projet](docs/software/build_flash.md). Le reflasher entre chaque rôle n'est pas nécessaire, à l'exception d'une modification des options de compilation comme `template`.

Les schémas custom restent centralisés dans `~/DEV/STM32/UAVCAN/DSDL/microcan`, branche **minican**. Les codecs partagés sont dans `~/DEV/STM32/UAVCAN/DSDLC`. Le TP ne nécessite pas de modifier ces schémas ni de les régénérer. Les détails de réception sont dans [DSDL/README.md](DSDL/README.md).

### Compilation des outils, si les exécutables manquent

À faire par le tuteur depuis la racine du projet, avec les dépendances déjà installées :

```sh
make -C ../TOOLS/uavcan_logger
make -C ../TOOLS/uavcan_param_editor
make -C ../TOOLS/uavcan_replay
cmake -S ../TOOLS/minican_tester -B ../TOOLS/minican_tester/build \
  -DLIBCANARD_DIR="$HOME/DEV/STM32/UAVCAN/libcanard" \
  -DDSDLC_DIR="$HOME/DEV/STM32/UAVCAN/DSDLC"
cmake --build ../TOOLS/minican_tester/build -j
```

Ces commandes construisent les outils du PC ; elles ne chargent pas de firmware dans la carte. Le tester nécessite notamment Qt Widgets et Qt SerialPort. Voir son [README](../TOOLS/minican_tester/README.md).

## 6. Dépannage rapide

| Symptôme | Vérifications dans l'ordre |
| --- | --- |
| Aucune carte dans la GUI | Alimentation, `can0`, débit, CAN_H/CAN_L/GND, terminaisons, allocation d'ID ; avec un ancien firmware, voir la récupération du mode identification au §2.3. |
| Carte visible en MAINTENANCE avec LED violette | Mode identification : régler `ROLE.identification=false`, sauvegarder et redémarrer pour lancer les autres rôles. |
| La carte apparaît puis redémarre | Chute de tension, courant limité, conflit/panne au démarrage, journal série et évolution d'`uptime`. |
| Un `ROLE.*` manque | Firmware ancien ou option `USE_*_ROLE` non compilée ; consulter le tuteur. |
| Paramètres perdus après redémarrage | Relire les valeurs acceptées, **Store All**, puis nouveau **Fetch All**. |
| Rôle activé mais inactif | Redémarrage effectué ? Autres rôles désactivés ? Capteur alimenté avant le démarrage ? |
| Message custom inconnu 20900/20901 | Relancer la GUI avec `--dsdl .../DSDL/microcan` ; utiliser les définitions partagées correspondant au firmware. |
| UART muet | Bon connecteur J3/J4, TX/RX croisés si UART à deux fils, masse, débit, port non occupé et inversion SBUS. |
| I2C muet | Module exact, tension, SDA/SCL, adresse, résistances de rappel ; débrancher la bande LED sur PB07. |
| Tester affiche PASS, appareil immobile | Lire le critère de PASS : PWM ne mesure aucun mouvement ; inspecter signal, câblage et alimentation. |
| Retour servo/moteur à `NaN` | Champ non mesuré/non fourni ou périmé ; vérifier les capacités du périphérique et la fiche du rôle. |
| Moteur continue après Stop Test | Couper sa puissance si nécessaire ; le rôle conserve sa consigne. Appliquer ensuite la séquence zéro de T14. |

## 7. Remise en état et fiche de résultats

1. Envoyer les consignes d'arrêt, vérifier l'immobilité puis couper les alimentations de puissance.
2. Arrêter le tester, les publications de console (`stop()`), les replays et le logger.
3. Dans la GUI, refaire **Fetch All**, puis **Restore from File** avec la sauvegarde initiale. Lire les incompatibilités éventuelles avant de poursuivre.
4. **Store All**, puis redémarrage avec les périphériques adaptés à la configuration restaurée. Si la sauvegarde réactive `ROLE.identification`, la carte corrigée reste accessible par CAN en mode MAINTENANCE avec sa LED violette ; les autres rôles restent arrêtés, sauf le shell si `ROLE.shell=true`.
5. À défaut de restauration validée, remettre tous les rôles d'actionnement à `false`, sauvegarder et laisser la carte hors tension, avec l'état consigné.
6. Conserver configurations, journaux, photographies et mesures dans un sous-dossier daté de `resultats_tests`.

Identification de l'essai :

| Information | À remplir |
| --- | --- |
| Date, stagiaire, tuteur | |
| Carte, révision matérielle, Node ID | |
| Version firmware / commit, débit CAN | |
| Versions GUI et outils | |
| Alimentation et limite de courant | |
| Modèles exacts des périphériques | |
| Sauvegarde de configuration initiale | |

Résultat possible : **OK**, **KO**, **partiel** ou **non testé**. « Partiel » signifie que certaines fonctions seulement ont été vérifiées ; préciser lesquelles. Un écart d'unité de message reste une non-conformité même si le capteur réagit.

| Test | Résultat | Mesures / fichier de preuve / anomalie |
| --- | --- | --- |
| T01 — Health survey | | |
| T02 — Tunnel série, deux sens et deux débits | | |
| T03 — SBUS simulé / récepteur réel | | |
| T04 — GNSS, messages / position 3D | | |
| T05 — MPL3115A2, acquisition / unités | | |
| T06 — QMC5883 | | |
| T07 — Microphone, silence / 1000 / 1500 Hz | | |
| T08 — OPT4060, périodique / événement / INT | | |
| T09 — VL53L4CX, trois distances / absence de cible | | |
| T10 — WS2812, couleurs / adressage | | |
| T11 — Voltmètre, tension / vitesse / expiration | | |
| T12 — PWM, CH1 à CH7 / OneShot125 | | |
| T13 — STS3032, mouvement / retour / unités | | |
| T14 — DShot, CH1 à CH4 / télémétrie / arrêt | | |
| T15 — Template, fréquences de journalisation | | |
| T16 — Identification, paramètres / redémarrage / ID dynamique / flash CAN | | |
| T17 — Shell, activation / commandes / mémoire / identification | | |
| Coexistence des capteurs et reprise éventuelle | | |
| Configuration remise en état | | |

Pour une anomalie, noter : **configuration utilisée → action réalisée → résultat attendu → résultat obtenu → preuve jointe**. Exemple : « MPL3115A2 seul ; ambiance 22 °C ; attendu environ 295 K ; reçu 22 dans `static_temperature` ; voir `barometre.log`. »

## 8. Références techniques du projet

- [Paramètres exacts](COMMON/source/nodeParameters.hpp) et [options de compilation](COMMON/source/roleConf.h).
- [Démarrage des rôles](microcan/source/UAVCanSlave.cpp) et [mode identification](microcan/source/main.cpp).
- [Brochage](microcan/cfg/MICROCAN.cfg), [schéma](HARDWARE/MICROCAN/MicroCan_v5_schematic.pdf) et [codes de LED interne](docs/software/onboard_rgb_led.md).
- [Documentation des capteurs microphone, lumière et distance](docs/software/roles/independent_sensors.md).
- [Télémétrie DShot](docs/software/roles/dshot_telemetry.md) et [configuration DShot compilée](COMMON/cfg/esc_dshot_config.h).
- [Présentation des outils TOOLS](../TOOLS/readme.txt) et [mode d'emploi du tester](../TOOLS/minican_tester/README.md).
- GUI locale : `~/DEV/STM32/UAVCAN/gui_tool/README.md` ; paramètres, console et panneaux vérifiés dans ses sources. Les anciennes descriptions de rôles peuvent être moins à jour que le firmware.
