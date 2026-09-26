# Rôle shell de diagnostic

`ROLE.shell` active la console de J3 / LPUART1 à **115200 bit/s, 8N1**.
Il vaut **false par défaut**. Dans DroneCAN GUI, régler `ROLE.shell=true`,
sauvegarder les paramètres puis redémarrer, même en mode identification.
La console démarre après le nœud UAVCAN. Les paramètres, le redémarrage et la
mise à jour CAN fonctionnent aussi lorsque le shell est désactivé.

Le changement s'applique au redémarrage, comme pour les autres rôles : il n'y a
pas de destruction des threads en cours d'utilisation. L'identification au boot
conserve le shell uniquement si `ROLE.shell=true`. Il ne force pas ce paramètre :
avec `ROLE.shell=false`, aucun contexte ni pile du shell ne sont alloués.
Tous les autres rôles restent suspendus, sans modification de leurs paramètres.
Si l'identification est activée après un démarrage normal, elle modifie seulement
la LED : le shell et tous les autres rôles déjà lancés continuent à fonctionner.

`NOSHELL=1` retire `TRACE`, le rôle et son paramètre à la compilation. Les
commandes shell passent sur l'UART physique, sans tunnel de console sur CAN.
`bus.serial.baudrate` concerne l'UART applicatif J4 et ne change pas ce débit.

## Branchement et commandes

Adaptateur USB-UART TTL 3,3 V : TX vers J3-7 (DBG_RX/PA03), RX vers J3-6
(DBG_TX/PA02), GND vers J3-1. Ne pas connecter son fil d'alimentation.
Appuyer sur Entrée pour afficher le prompt. Les commandes comprennent `info`,
`mem`, `threads`, `uid`, `adc`, `st`, `uavp`, `can`, `param`, `restart` et `panic`.
`threads` dépend de `CH_DBG_STATISTICS`. `st` est le nom effectif de la commande
de stockage, par exemple `st ROLE.shell false`, puis `restart` pour désactiver
la console. L'historique et la complétion Tab restent disponibles.

L'activation étant tardive, les traces précédant le démarrage du rôle ne sont
pas mémorisées. Si le stockage ou les abonnements d'un autre rôle empêchent ce
démarrage, la console ne sera pas disponible. Elle n'est donc plus une voie de
secours systématiquement présente dès la mise sous tension.

## Allocation mémoire

Les tailles suivantes proviennent du build ARM habituel `-Og`, statistiques
activées ; elles peuvent changer avec la configuration de ChibiOS.

| Allocation à l'activation | Taille demandée | Allocateur |
| --- | ---: | --- |
| Objet `ShellRole` | 8 octets | `new`, heap applicatif |
| Contexte (éditeur microrl, historique, complétion, statistiques, calibration) | 1256 octets | `malloc_m`, heap applicatif |
| Thread `Enhanced_shell`, pile de 2000 octets et contexte RTOS | 2480 octets | heap ChibiOS par défaut |
| Thread `serialPrint`, pile de 2048 octets et contexte RTOS | 2528 octets | heap ChibiOS par défaut |
| Total | **6272 octets** | hors en-têtes et alignement des allocations |

Ces allocations n'ont pas lieu lorsque `ROLE.shell=false`, y compris en mode
identification. Avec `ROLE.shell=true`, elles ont lieu dans les deux modes.
La pile du shell était déjà dynamique auparavant ; les
réserves statiques de l'éditeur, des statistiques et du thread d'impression
ont été supprimées. La bibliothèque microrl partagée reste utilisée pour
l'édition des lignes, avec un contexte appartenant à ce projet. Les fonctions
de formatage partagées sont conservées ; leur ancien thread statique est exclu
du binaire par l'élimination des sections inutilisées.

Il reste un coût fixe : quelques pointeurs/mutex, le petit état interne du
parseur microrl et les tables de chaînes, ainsi que le pilote HAL `LPSD1`
(**96 octets**) et ses buffers RX/TX (**128 octets chacun**). Le formateur
partagé conserve également son état newlib (288 octets dans ce build).
Le pilote est initialisé par le HAL mais son périphérique UART est démarré
uniquement par le rôle. Le coût RAM désactivé n'est donc pas nul.

Une erreur d'allocation du contexte ou des piles annule le démarrage : mémoire
créée pour cette tentative libérée, UART arrêté si nécessaire, réservation
LPUART1/PA02/PA03 rendue et statut `SHELL_ROLE/HEAP_FULL` remonté. Les appels
aux traces restent inactifs tant que le shell n'a pas complètement démarré.

`mem` affiche le heap applicatif, le heap des threads et la réserve mémoire
centrale. Le total BSS affiché par `size` n'est pas une mesure suffisante du gain :
le script de liaison réserve aussi la RAM restante au heap. Pour comparer deux
configurations au débogueur, relever les heaps **après le démarrage**, avec les
mêmes autres rôles, puis comparer `ROLE.shell=false` et `true` après reboot.

## Vérification

```sh
python3 tests/identification/run.py
python3 tests/shell/run.py
make -C microcan -j4 firmware
python3 tests/shell/check_binary.py microcan/build_MICROCAN/MICROCAN.elf
make -C microcan -j4 NOSHELL=1 BUILDDIR=/tmp/microcan-shell-noshell DEPDIR=/tmp/microcan-shell-noshell-dep firmware
python3 tests/shell/check_binary.py --noshell /tmp/microcan-shell-noshell/MICROCAN.elf
```

Les tests hôtes vérifient le choix des rôles, l'isolation en identification,
les échecs d'allocation et le nettoyage, le filtrage des commandes, la complétion
et les bornes du formatage. Le contrôle ELF détecte la réapparition des anciennes
réserves statiques. Les essais réels de terminal, d'historique, de CAN et de
mémoire sont décrits dans la fiche **T17** de
[procedure_de_test.md](../../../procedure_de_test.md).
