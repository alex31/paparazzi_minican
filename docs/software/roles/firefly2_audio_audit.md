# Audit sonore FireFly II — 22 septembre 2026

Le simulateur reproduit correctement le motif sonore enregistré. Le traitement
MINICAN présente cependant des défauts reproductibles avec ce son continu.
Ces résultats ne permettent pas d'attribuer le mauvais largage à la MINICAN :
les paramètres effectifs et les données du vol ne sont pas connus dans cet
audit. L'utilisateur confirme que la version flashée est celle des fichiers
actuels de ce répertoire, construite avec `make`. Le signal lumineux n'a pas
été étudié.

## Versions et méthode

- Vidéo : `docs/VID_20260921_093815.mp4`, SHA-256
  `9c1c1d52352d0562c496808c5a4b66642d36d3db7186daabea2485275bc9dd00`.
- Simulateur : `C21DEV/DEVBOARDH7/devbh7_shell`, fichiers présents au moment de
  l'audit, profil `firefly2`.
- Référence historique MINICAN, distincte de la version flashée : commit
  `0b7d23c7b73c9216e59bd602fd6b54164596d4c9`.
- Version « actuelle » ci-dessous, confirmée comme version flashée : arbre de
  travail trouvé à l'ouverture de l'audit, contenant déjà le correctif non
  commité de publication continue à
  200 ms. SHA-256 de `COMMON/source/imavRole.cpp` :
  `f010f2a71634ec47b0eecf04f7ec836136e3c596ba560d9fa4cecb4444d60fb9`.

Le `Makefile` racine lance `make -C microcan PLATFORM=MICROCAN ... firmware`.
`microcan/Makefile` inclut `../COMMON/source/imavRole.cpp` et
`COMMON/source/roleConf.h` active `USE_IMAV_ROLE`. La compilation utilise bien
les fichiers locaux, y compris le correctif non commité de publication à
200 ms ; le commit Git seul ne décrit donc pas cette version embarquée.
L'ELF existant `microcan/build_MICROCAN/MICROCAN.elf`, daté du 21 septembre
2026 à 10:11:18 (heure locale), contient aussi `publishAudioSnr()`,
`finalizeAudioSnr()` et la branche de publication continue à 4 000 ticks,
soit 200 ms. Le correctif est donc présent dans le binaire local examiné.

Le C++ de production est compilé sur PC avec l'adaptateur du test existant
`tools/imav_monitor/tests/test_audio_publication.py`. Les fonctions DSP,
l'automate et la publication/expiration sont conservés. L'ancien nom
`publishAudioBurst` est seulement renommé en `publishAudioSnr` dans la copie
temporaire pour réutiliser cet adaptateur.

Le son décodé est rééchantillonné à 23 998 Hz et converti en ADC synthétique
par `clamp(round(4096 + 3000*x), 0, 8191)`. L'horloge, le matériel, le RTOS et
le transport CAN sont simulés ; ce rejeu ne mesure donc ni la sensibilité
acoustique réelle, ni la charge CPU embarquée, ni les pertes CAN. Les temps
sont relatifs au début du son décodé, dont l'origine est à 11,292 ms dans le
conteneur vidéo. Les paramètres sont `band_low_hz=2000`, `snr_alpha=1`, sauf
mention contraire.

## Son enregistré et simulateur

La fondamentale est estimée par passages à zéro interpolés après filtrage
1,6–2,8 kHz, sur 20 périodes acoustiques. Pour le régime établi de la vidéo
après 9 s :

| Grandeur | Vidéo | WAV du simulateur |
| --- | --- | --- |
| Fondamentale estimée | 1 984–2 255 Hz | 1 987–2 258 Hz |
| Période moyenne du motif | 270,094 ms, 25 intervalles | 270,081 ms, 18 intervalles |
| Écart type des périodes estimées | 0,248 ms | 0,177 ms |

Le son reste présent entre les répétitions du motif. Sa modulation ne produit
pas les blocs spectraux faibles nécessaires pour séparer des salves dans le
détecteur actuel.

Le simulateur utilise exactement 21 608 échantillons à 80 kHz, soit une période
numérique de **270,1 ms / 3,70233 Hz**. L'extrait commence à 11,917342 s dans la
vidéo ; il est converti en mono, filtré à 1–10 kHz, normalisé et raccordé sur
1 ms. La lecture DAC/DMA est continue, au gain numérique 0,30.

Vérifications effectuées : empreinte de la vidéo identique à la référence du
simulateur ; régénération dans `/tmp` donnant PCM, WAV et JSON identiques
octet pour octet ; deux tests simulateur réussis, dont le pilote `audio.cpp`
avec DMA simulé sur 320 cycles et arrêt/redémarrage. Aucun défaut numérique
de génération ou de lecture identifié.

La boucle ne représente toutefois qu'un cycle, parfaitement répété, capté
par le téléphone et filtré. Elle n'est pas calibrée en pression acoustique et
ne reproduit pas le haut-parleur réel, les déplacements ou le bruit des moteurs.

## 1. Défaut actuel : démarrage ou reprise pendant la sirène

Dans `COMMON/source/imavRole.cpp`, `updateAudioCadence()` remet l'automate en
`Unarmed` après une discontinuité. Pour en sortir, il exige deux blocs
consécutifs de score inférieur ou égal à 0,30, soit environ 42,7 ms de faible
signature spectrale. Un son continu déjà reconnu spectralement empêche cette
transition ; les blocs suivants ne peuvent jamais ouvrir une première salve.

| Rejeu avec la version actuelle | Résultat |
| --- | --- |
| Vidéo entière, comprenant le début calme | 50 `snr` positifs, de 6,2512 à 16,044 s |
| Même vidéo, écoute démarrée à 7 s | Aucun `snr` positif pendant les 9,09 s restantes |
| Vidéo entière, une discontinuité injectée à 9 s | Dernier positif à 8,85405 s ; zéro à 10,41155 s ; aucune reprise jusqu'à la fin |
| WAV simulateur déjà actif au démarrage | Aucun `snr` positif pendant 5,40 s |
| Même WAV précédé d'une seconde de silence | 26 `snr` positifs |

Le worker `audioThread()` marque effectivement une discontinuité lorsqu'il
constate un saut de compteur DMA, puis appelle ce même automate. Le test
injecte ce marqueur : il démontre sa conséquence logique, sans prouver qu'une
perte de bloc s'est produite en vol. L'absence d'armement au démarrage est
également reproduite en commençant la vidéo à 6,5, 8, 10, 12 ou 14 s.

**Correction à prévoir :** permettre l'acquisition et la réacquisition d'un
son continu sans exiger un silence préalable, avec une politique explicite
d'initialisation du plancher de bruit. Ajouter ces cas aux régressions avant
validation sur carte ; les tests existants vérifient actuellement qu'une
discontinuité exige un réarmement, ce qui entérine le comportement inadapté.

## 2. Comparaison historique : publication après extinction, déjà corrigée

Le commit de référence ne publie `snr` que lorsque la salve se termine.
La nouvelle sirène ne produit pas cette fin de salve pendant l'enregistrement.

| Version | Vidéo seule | Vidéo suivie de 3 s de silence |
| --- | --- | --- |
| Commit `0b7d23c` | Aucun `snr` positif | Un seul positif à 16,108 s, après arrêt du son |
| Arbre de travail actuel | 50 positifs à environ 5 Hz | Dernière fenêtre positive à 16,108 s, puis zéro à 17,8148 s |

Le correctif local préexistant, `continuousAudioPublishPeriod` et
`finalizeAudioSnr()`, corrige cette publication en continu. Il ne corrige pas
le défaut d'armement précédent. Puisque la version flashée correspond aux
fichiers actuels, le défaut historique de publication seulement après
extinction est écarté des causes possibles pour ce vol.

## 3. Sélectivité et valeur du SNR pour la localisation

Le code valide surtout une énergie concentrée dans la bande sélectionnée,
supérieure à quatre références latérales. Il ne vérifie ni le balayage de la
nouvelle sirène, ni sa répétition à 3,70 Hz. `cadenceHz` est diagnostique et ne
conditionne pas la publication.

Un **simple sinus continu de 2 250 Hz**, après une seconde de silence, produit
15 publications positives sur trois secondes de son. Le détecteur accepte
donc aussi un autre son tonal dans la bande. C'est une limite de sélectivité
démontrée, pas la preuve d'un faux positif pendant le vol.

Le `snr` utilise un plancher appris avant le son reconnu. Ce plancher reste
figé tant que l'automate est `On` (`analyzeAudioBlock()`). Dans le test
`current_increasing_noise`, un sinus constant de 2 250 Hz/amplitude 0,2 est
présent à partir de 1 s. Le bruit blanc gaussien passe d'un écart type 0,005 à
0,12 à 3 s, avec graine 101. Le rapport physique signal/bruit baisse de
27,6 dB ; le SNR publié moyen passe pourtant de **40,12 à 41,21 dB**.

Cette valeur représente donc une puissance relative à un fond antérieurement
appris, avec maximum sur 200 ms ; elle n'est pas une estimation indépendante
du bruit courant, ni une distance à la balise. Les 50 valeurs du rejeu vidéo
sont toutes plafonnées à 60 dB avec le gain de cet adaptateur. Ce plafonnement
logiciel ne démontre pas une saturation du microphone en vol, mais empêche ce
test de valider une recherche de maximum spatial.

## 4. Bande, acquisition et paramètres

La bande par défaut convient : 2–3 kHz avec garde de 50 Hz, soit 1 950–3 050 Hz
pour les bins utilisés. Le rejeu de la vidéo avec `band_low_hz=2600` ne produit
aucune valeur positive ; les essais à 2 400 et 2 500 Hz échouent également.
L'export JSON local indique 2 000 Hz, sans garantir la valeur effectivement
sauvegardée sur la carte. Ce paramètre n'est lu qu'au démarrage du rôle.

L'arithmétique d'acquisition vérifiée est cohérente : timer à 42,5 MHz divisé
par 1 771, soit 23 997,741389 Hz ; demi-buffer de 512 échantillons, soit
21,335 ms ; ADC suréchantillonné ×4 et décalé d'un bit, donnant l'échelle
13 bits attendue. Les 31 coefficients Goertzel correspondent à cette fréquence
d'échantillonnage et la fenêtre Hann est cohérente. Cela ne constitue pas une
mesure du respect des échéances sur le processeur embarqué.

## Ce que valide imav_monitor

`tools/imav_monitor/src/detection_state.cpp` transforme **tout `snr > 0` en
100 %**. La valeur numérique en dB reste affichée séparément. Le vert valide
la réception récente d'une valeur positive ; il ne quantifie ni la force, ni
la spécificité du son, ni la qualité de sa localisation.

La dernière valeur peut rester positive environ 1,5–1,72 s après sa dernière
publication avant expiration firmware. Sans nouvelle réception, le moniteur
la rend indisponible après 2 s. Le message ne contient pas de timestamp audio
MCU : l'interface date sa réception sur le PC. Aucun défaut de décodage CAN
n'a été identifié ; le test C++ `detection_state_test` passe.

## Reproduire l'audit

Depuis la racine MINICAN, avec Python, g++ et ffmpeg :

```sh
python3 tools/imav_monitor/audit_audio_beacon.py \
  --compare-ref 0b7d23c \
  --output /tmp/imav-audio-audit
python3 -m unittest discover -s tools/imav_monitor/tests -p test_audio_publication.py
```

Le premier outil écrit `summary.json` et les mesures CSV ; il conserve les
empreintes des entrées et sources. Le second exécute les sept tests
préexistants, tous réussis lors de cet audit. Leurs scénarios favorables ne
couvrent pas les échecs documentés ci-dessus. Aucun firmware n'a été modifié
ou flashé pendant l'audit.

Pour poursuivre le diagnostic du largage, les éléments les plus utiles sont
la valeur effective de `band_low_hz`, puis une capture CAN avec `snr`, `a0`,
`aud`, `p0` et `frq`, corrélée à la trajectoire. Un essai au sol avec sirène
déjà active avant le démarrage de la MINICAN cible directement le défaut
d'armement ; une comparaison moteurs arrêtés/allumés permettra ensuite de
tester la sélectivité et le suivi du niveau dans la chaîne acoustique réelle.
