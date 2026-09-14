# Essais lumineux avec la frontale CREE

Le paramètre booléen UAVCAN `role.imav.light.cree_test` est désactivé par
défaut. L'activer sélectionne un profil réservé aux essais avec la frontale
CREE, blanche ou munie d'un cache rouge.

## Mesure vidéo du 11 septembre 2026

Source : `VID_20260911_214652.mp4`, 1 032 images sur environ 8,59 s,
enregistrées à environ 120 images/s. Les horodatages des images ont été
utilisés directement pour mesurer les transitions.

| Mesure | Résultat |
| --- | --- |
| Fréquence, ajustement sur 68 fronts montants | 7,99 Hz |
| Période | environ 125,15 ms |
| Rapport cyclique | proche de 50 % |
| Durées allumé / éteint retenues par le firmware | 62 / 63 ms |

La fréquence est cohérente entre plusieurs zones de l'image. La largeur des
impulsions est moins précise : sur deux zones éclairées hors du centre
saturé, elle vaut environ 61 à 67 ms selon le seuil de mesure. L'exposition,
le traitement de la caméra et l'intervalle de 8,3 ms entre images limitent
la précision. Les durées 62/63 ms représentent donc un profil nominal à
8 Hz et environ 50 %, pas une mesure optique à la milliseconde.

## Utilisation

1. Activer `ROLE.imav.beacon` et `role.imav.light.cree_test` dans les
   paramètres UAVCAN, enregistrer, puis redémarrer la carte.
2. Utiliser le mode clignotant de la frontale. Le cache rouge est facultatif
   pour ces essais. Le détecteur accepte l'intensité RGB moyenne et conserve
   les contrôles de cadence, de cohérence et de rapport H2/H1. La phase H2
   est ignorée dans ce profil, car cette harmonique disparaît presque à 50 %.
   Une modulation minimale est exigée : le facteur de score passe de zéro
   à un entre 0,5 % et 2 % d'amplitude relative du fondamental. Cela évite
   qu'un éclairage constant intense et le bruit du capteur entretiennent
   une détection.
3. Observer `lit`. Pour le diagnostic, activer
   `role.imav.debug.publish.optional` : `lhz` doit se situer autour de 8 Hz
   et `lpt` vaut 3 lorsqu'un motif CREE est sélectionné. La détection utilise
   une fenêtre d'environ une seconde. `lsa` indique les saturations ; une
   mesure saturée est rejetée et réinitialise cette fenêtre.
4. Avant la compétition, remettre `role.imav.light.cree_test` à `false`,
   enregistrer et redémarrer. Les paramètres MotionSCOUT stockés redeviennent
   actifs ; conserver `role.imav.light.beginning_pattern=false` pour le seul
   motif établi.

Le mode CREE remplace temporairement `high_ms`, `steady_low_ms` et la banque
de démarrage ; il ne modifie pas leurs valeurs enregistrées. Il est lu au
démarrage du rôle, comme les autres paramètres de cadence lumineuse.

Ces essais permettent de vérifier la chaîne capteur/détection/UAVCAN avec
la frontale. La portée et les seuils du profil compétition restent à valider
avec la balise MotionSCOUT réelle.
