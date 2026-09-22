# Essais lumineux FireFly II avec la DevBoardH7

Le profil FireFly II de `DEVBOARDH7/devbh7_shell` reproduit la vidéo
`docs/VID_20260921_093815.mp4` : les deux premières LED du ruban clignotent
ensemble en rouge pendant 55 ms toutes les 270,1 ms, soit 3,702 Hz.
Le son reste continu et ne commande pas l'extinction des LED.

Les mesures vidéo donnent une période lumineuse moyenne de 269,87 ms et
une durée allumée médiane de 53,80 ms (moyenne 56,37 ms). La résolution de
la vidéo limite la précision des transitions à environ 17 ms. Le profil
du simulateur est donc cohérent avec cet enregistrement.

## Réglages MicroCAN

Le détecteur standard est configurable mais ses valeurs par défaut visent
MotionSCOUT : 100 ms allumé, 233 ms éteint, environ 3 Hz. Sa recherche autour
de cette fréquence ne couvre pas le profil FireFly II à 3,70 Hz.

Dans les paramètres UAVCAN, appliquer puis enregistrer et redémarrer :

| Paramètre | Valeur pour cet essai |
| --- | --- |
| `ROLE.imav.beacon` | `true` |
| `role.imav.light.high_ms` | `55` |
| `role.imav.light.steady_low_ms` | `215` |
| `role.imav.light.cree_test` | `false` |
| `role.imav.light.beginning_pattern` | `false` |

Le paramètre `adaptive_pattern` peut rester désactivé : il apprend les
triplets et les flashs lents d'une autre vidéo, pas les flashs réguliers
FireFly II. L'activer seul ne corrige pas les durées du détecteur standard.
Le mode CREE doit être désactivé car il remplace les durées stockées par
son propre profil 8 Hz. Les durées sont lues au démarrage du rôle.

Pour revenir au profil MotionSCOUT, restaurer `high_ms=100` et
`steady_low_ms=233`, enregistrer et redémarrer. Ces réglages sélectionnent
un profil ; ils n'ajoutent pas une détection simultanée des deux balises.

## Vérification sur PC et sur carte

Le rejeu du traitement lumineux C++ de la MicroCAN sur un signal RGBW
synthétique reproduisant 55/215,1 ms, avec acquisitions espacées de 7,2 ms,
donne :

| Configuration du détecteur | Résultat |
| --- | --- |
| 100/233 ms | aucun événement `lit`, score maximal 0,371 |
| 100/233 ms + adaptatif | aucun événement `lit`, score maximal 0,371 |
| 55/215 ms | 60 événements sur environ 17,3 s ; premier à 1,10 s ; verrouillage maintenu après acquisition |

Les intervalles entre événements du profil corrigé sont de 266,4 à
273,6 ms, autour des 270,1 ms de la source. Les deux tests PC du simulateur
passent aussi, dont la phase optique issue du vrai pilote audio avec DMA
simulé sur 320 cycles.

Ces résultats valident la compatibilité logicielle des profils ; ils ne
mesurent pas la lumière émise par le ruban ni les paramètres déjà stockés
sur une MicroCAN. Pour le banc, activer temporairement
`role.imav.debug.publish.optional` et observer :

- `lit` positif à environ 3,7 Hz après acquisition ;
- `lhz` proche de 3,70 Hz et `lpt=2` (voie spectrale standard configurée) ;
- `lrd/lgn/lbl` montrant les flashs rouges ;
- `lsa`, `ler` et `lgp` pour distinguer saturation, erreurs et interruptions
  de mesure d'une incompatibilité de motif.

Le seul paramétrage suffit au rejeu : aucune modification du motif produit
par le simulateur ni de l'algorithme de détection n'est nécessaire.
