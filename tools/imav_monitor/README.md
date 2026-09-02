# IMAV monitor

Application Qt/C++ de suivi de la détection de la balise MotionSCOUT. Elle lit
directement `can0` avec SocketCAN et décode DroneCAN v0 avec `libcanard`. Elle
n'utilise pas la console série de la MicroCAN.

L'interface affiche :

- la détection sonore issue de `det`, avec le `snr` relatif en dB ;
- le score lumineux `lit` ;
- la corroboration des deux capteurs, calculée par `det × lit` ;
- les trois courbes sur une fenêtre glissante de 30 secondes.

Une valeur devient indisponible si sa clé n'a pas été reçue depuis une seconde.
Les coupures apparaissent alors comme des trous dans le graphe, au lieu d'être
interprétées comme une absence de balise.

## Préparation de CAN

Le firmware et le dongle utilisent du CAN classique à 1 Mbit/s :

```sh
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 1000000 fd off
```

Le rôle `ROLE.imav.beacon` doit être actif. `det`, `snr` et `lit` sont émis
systématiquement à 5 Hz ; il n'est pas nécessaire d'activer
`role.imav.debug.publish.optional`.

## Compilation

Dépendances : CMake, Qt Widgets 5 ou 6, les en-têtes Linux SocketCAN,
`libcanard` et les sources C générées par DSDLC. Les chemins par défaut sont les
mêmes que ceux des applications présentes dans `MINICAN/TOOLS` :

```sh
cd tools/imav_monitor
./dobuild
```

Ils peuvent être remplacés si nécessaire :

```sh
./dobuild \
  -DLIBCANARD_DIR=/chemin/vers/libcanard \
  -DDSDLC_DIR=/chemin/vers/DSDLC
```

## Exécution

Le nœud MicroCAN vaut 10 par défaut :

```sh
./build/imav_monitor
```

Options utiles :

```text
--interface can0
--source-node 10
--history 30
```

L'application est passive : elle ne réserve pas d'identifiant de nœud UAVCAN
et n'émet aucune trame sur le réseau.

## Enregistrement pour post-traitement

`record_imav_can.py` sauvegarde sans agrégation tous les
`uavcan.protocol.debug.KeyValue` du nœud sélectionné, y compris les valeurs
nominales et optionnelles. Chaque ligne contient l'heure UTC et nanoseconde de
l'hôte, le temps monotone depuis le début, le nœud source, le transfer-ID, la
clé et sa valeur flottante.

Avec `role.imav.debug.publish.optional=true`, les diagnostics dérivés restent à
5 Hz. Les sept clés optiques lossless suivent le data-ready, jusqu'à environ
139 Hz, et leurs groupes se reconstruisent avec `lct`; `ltu` donne leur temps
MCU en microsecondes
modulo 2^24. `lrd/lgn/lbl/lwh` contiennent les canaux RGBW bruts linéarisés
nécessaires aux FFT, filtres adaptés et autres post-traitements.

Le firmware publie aussi à 5 Hz `lfs` (voie rapide), `lsc` (voie spectrale
lente), `lsn` (proéminence locale en dB), `lco` (cohérence), `lrf` (fraction
rouge périodique), `lfq`
(fréquence du pic) et `lhr` (rapport deuxième harmonique/fondamental). Ces
grandeurs permettent de recaler les seuils sur une capture extérieure sans
modifier la sortie nominale `lit` à 5 Hz.

Capture jusqu'à `Ctrl-C` :

```sh
./record_imav_can.py --interface can0 --source-node 10
```

Les fichiers automatiques sont placés dans `captures/`. Une durée et un nom
explicites peuvent également être donnés :

```sh
./record_imav_can.py --duration 60 --output /tmp/imav_50cm.csv
```
