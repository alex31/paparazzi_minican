# Profil lumineux adaptatif MotionSCOUT

Cette option **conserve le détecteur spectral standard** et ajoute en
parallèle la reconnaissance des nouveaux motifs de la vidéo. Elle ne remplace
ni l'algorithme existant, ni ses seuils, ni sa rapidité sur les motifs connus.
La périodicité seule ne valide pas une lumière. L'extension est désactivée
par défaut ; le mode CREE reste séparé.

## Choix sur le terrain

| Paramètre UAVCAN | Standard + vidéo | Standard seul |
| --- | --- | --- |
| `role.imav.light.adaptive_pattern` | `true` | `false` |
| `role.imav.light.cree_test` | `false` | `false` |

Enregistrer puis redémarrer la MicroCAN, avec `ROLE.imav.beacon` actif.
Aucun réglage de durée n'est nécessaire pour les nouveaux motifs. Les
paramètres `high_ms`, `steady_low_ms`, `beginning_low_ms` et
`beginning_pattern` restent effectifs pour la voie standard. En particulier,
la préalarme à 2 Hz conserve sa sélection par `beginning_pattern` : l'extension
ne force pas son activation. Si `cree_test=true`, CREE reste prioritaire et
l'extension vidéo est désactivée.

## Motifs couverts

| Famille | Domaine de reconnaissance | Référence nominale |
| --- | --- | --- |
| Préalarme régulière | Détecteur et réglages standard, si `beginning_pattern=true` | 100 ms allumé, 400 ms éteint |
| Alarme régulière | Détecteur et réglages standard | 100 / 233 ms |
| Trois flashs + pause | Trois allumages de 50–200 ms, deux pauses de 150–300 ms et une de 500–900 ms ; cycle de 1,1–1,8 s | 100 / 220, 100 / 220, 100 / 680 ms |
| Clignotement lent de la vidéo | 1,3–1,6 Hz ; allumage 220–350 ms, extinction 300–550 ms | 270 / 420 ms |

La vidéo `Dead_man_system.mp4`, 18,28 s à environ 60 images/s, montre les
triplets entre environ 5 et 11,5 s puis le rythme lent pendant les
manipulations. Ces observations ne constituent pas une spécification MSA.
La voie ajoutée n'accepte que les triplets et le rythme lent, jamais les
rythmes réguliers à 2/3 Hz déjà traités par la voie standard. Elle ne peut
donc pas remplacer leur détection par un apprentissage plus lent. Le voyant
à 1 Hz, CREE à 8 Hz et les autres familles sont rejetés par cette voie.

La qualification rouge/contraste est conservée. Pour la voie ajoutée, le score utilise
une plage locale d'intensité sur 1,6 s, échantillonnée toutes les 20 ms, en
écartant les deux extrêmes de chaque côté. Cela évite qu'un pic isolé ou le
plancher lent standard fragmente les flashs. Une modulation minimale et une
dominante rouge restent nécessaires ; les flashs blancs seuls sont rejetés.
Deux mesures successives confirment chaque transition. Trois répétitions des
temps hauts/bas doivent concorder : dispersion maximale de 30 ms ou 20 %
de la durée moyenne, selon la valeur la plus grande. L'historique contient
au plus neuf couples haut/bas ; la fenêtre spectrale standard reste à 1 s.
La plage d'intensité utilise un tampon fixe d'environ 1,3 ko, sans allocation
dynamique supplémentaire.

Seule l'acquisition des nouveaux motifs nécessite un premier flash puis
trois cycles complets : environ 2,4 s pour le rythme lent et 4,4 s pour les
triplets, à partir du premier flash correctement observé. Un
changement de motif ou un flash manquant peut nécessiter un nouvel
apprentissage. Une pause longue conforme conserve la détection.

## Sorties et vérification

Tant qu'aucun nouveau motif n'est acquis, `lit` conserve les scores et les
événements du détecteur standard. Après acquisition d'un motif vidéo,
ses fronts descendants **observés** deviennent la source de `lit`, avec
confirmation par deux mesures et sans doublon à la prise de relais. Aucun
événement spectral extrapolé n'est publié dans ses pauses. Les deux
détecteurs continuent de fonctionner ; perdre une voie ne publie pas zéro
si l'autre reconnaît toujours un motif. Le retour au standard est automatique.
Un trou d'acquisition de 100 ms réinitialise l'apprentissage. Le délai de
secours vaut 1,8 s pendant un motif vidéo acquis, et conserve la valeur
standard sinon. Les zéros inactifs sont répétés à 1 Hz.

Avec `role.imav.debug.publish.optional=true` :

- `lpt=4` : nouveau motif vidéo acquis ; sinon les codes standard `0/1/2/3`
  restent effectifs ;
- `lpn` : nombre de flashs par cycle appris, `1` ou `3` ;
- `lhz` : fréquence du **cycle complet**, environ 0,70 Hz pour les triplets ;
- `lon/lof` : dernières durées haute/basse acceptées ;
- `lcs/lts` : cohérence temporelle ; `lfs/lps` : score appris ;
- `lis/lac/lrr` : diagnostics standard, sauf pendant `lpt=4`, où ils
  décrivent le score/contraste/couleur de la voie ajoutée.

Les tests PC compilent le même détecteur C++ que le firmware :

```sh
cmake -S tools/imav_monitor -B tools/imav_monitor/build
cmake --build tools/imav_monitor/build
ctest --test-dir tools/imav_monitor/build --output-on-failure
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s tools/imav_monitor/tests -p 'test_*.py'
```

Ils couvrent les familles admises, les temps extraits de la vidéo, les
événements réels sans publication dans les pauses, l'extinction, les motifs
rejetés, la qualification RGB (rouge, blanc, vert, bleu, faible modulation),
le réapprentissage et le débordement du compteur temporel. Le test
`test_light_extension.py` compile les méthodes de traitement et de publication
du firmware sur PC (horloge, CAN et accès matériel simulés) et compare les
scores/états/événements avec l'extension activée et désactivée. Il vérifie
aussi les pauses, le retour au standard et l'isolation CREE.
`offline_light_detector.py` reste destiné aux profils spectraux standard et
CREE. Les tests adaptatifs utilisent des RGB/score synthétiques et les temps
vidéo, pas une acquisition OPT4060 de cette balise. Un rejeu complémentaire
des six captures RGBW existantes, avec la banque de démarrage désactivée puis
activée, donne des scores, états et événements CAN identiques au firmware
standard d'origine. Ces captures locales sont également rejouées par les
tests lorsqu'elles sont disponibles.
La portée et les faux positifs restent à vérifier sur la carte ; une autre
lumière rouge reproduisant un motif admis peut aussi être reconnue.
