# Messages CAN du rôle IMAV 2026

## Objet et périmètre

Le rôle `ROLE.imav.beacon` publie deux scores destinés au contrôleur de vol :
force sonore relative et score de détection lumineuse.
Elles constituent l'interface CAN opérationnelle du rôle pour l'épreuve IMAV
2026.

Toutes les autres clés décrites en annexe sont des données d'instrumentation.
Elles servent à enregistrer les capteurs, mettre au point les algorithmes et
expliquer une détection ou un faux positif. **Elles ne doivent pas être
consommées par la logique de mission pendant l'épreuve IMAV 2026.** Leur
format, leur cadence et même leur présence peuvent évoluer pendant la mise au
point.

## 1. Transport de l'interface opérationnelle

Les deux valeurs sont diffusées individuellement sous forme de messages
UAVCAN v0 `uavcan.protocol.debug.KeyValue` (Data Type ID 16370), avec une
priorité CAN basse :

- réseau CAN classique à 1 Mbit/s, sans CAN FD ;
- clé ASCII de trois caractères ;
- valeur `float32` ;
- `snr` une fois par salve sonore reconnue ; `lit` une fois par flash lumineux
  reconnu, soit environ 3 Hz en régime établi ;
- publication broadcast : aucun acquittement ni abonnement préalable ;
- le Node-ID UAVCAN source identifie la MicroCAN qui porte les capteurs.

`snr` est envoyé lors de la transition qui clôt une salve reconnue. `lit` est
émis directement par le thread optique sur le front descendant estimé du
flash, après recalcul du score avec le dernier groupe RGBW. Après expiration,
chaque producteur publie zéro puis le répète à 1 Hz tant que son score reste
inactif.

Chaque clé est un transfert UAVCAN indépendant. Les deux valeurs ne forment
pas un paquet atomique. Le récepteur mémorise la dernière valeur de chaque
couple `(Node-ID, clé)` ; les zéros répétés invalident les scores périmés même
si une trame isolée est perdue.

## 2. Messages opérationnels

| Clé | Cadence | Unité/domaine | Signification opérationnelle |
|---|---:|---|---|
| `snr` | par salve, puis zéro à 1 Hz | dB relatifs ou `0.0` expiré | Pic de la salve sonore qui vient de se terminer, au-dessus du plancher de bruit adaptatif dans la bande 2–3 kHz. Il passe à zéro après 1,5 s sans salve reconnue. |
| `lit` | par flash, puis zéro à 1 Hz | score continu de `0.0` à `1.0` | Confiance lumineuse du profil sélectionné : fenêtre spectrale de 1 s ou motif temporel appris. Émise au front descendant estimé (standard/CREE) ou observé (adaptatif). |

### 2.1 Interprétation de `snr`

`snr` est la grandeur principale pour comparer la force sonore en différents
points de la trajectoire et diriger le drone vers la balise. Elle est relative
au bruit moteur appris localement. Elle convient donc à une cartographie
spatiale ou à une recherche de gradient avec la même MicroCAN.

Le filtre IIR entre pics successifs est réglé à chaud par
`role.imav.audio.snr_alpha`, entre 0,5 et 1. La valeur par défaut 1 publie le
pic brut du nouveau chirp et n'ajoute aucun retard de lissage ; 0,5 reproduit
l'ancien mélange moitié ancienne mesure, moitié nouveau pic. Après 1,5 s sans
salve reconnue, l'émetteur remet également cet historique IIR à zéro.

Ce n'est pas une mesure acoustique absolue. Il ne faut pas comparer directement
deux MicroCAN sans calibration, ni convertir `snr` en distance avec une loi
universelle. L'orientation du microphone, le vent, le régime moteur et les
réflexions acoustiques modifient la valeur.

### 2.2 Interprétation de `lit`

`lit` est également un score continu, pas un booléen et pas une mesure de lux.
Il est exactement le score du détecteur rapide, également exposé sous `lfs`
quand le débogage est actif. En mode standard, le détecteur analyse une fenêtre glissante d'une
seconde du contraste rouge avec cinq couples fondamental/H2. Cohérence,
couleur, rapport H2/H1 et phase relative qualifient ensemble le motif de la
balise sans seuil de luminosité absolue.

Cette mémoire unique et finie évite tout basculement vers un retard long. Son
centre temporel se trouve environ 0,5 s avant le calcul. Le coefficient
complexe du bin central fournit la phase absolue ; le firmware publie une seule
fois par cycle dans les 20 ms suivant le front descendant prévu. Le score est
recalculé à cet instant, ce qui supprime la gigue de 0 à 200 ms de l'ancienne
publication périodique. Le verrouillage utilise une hystérésis 0,55/0,30 et
émet immédiatement `lit=0` lorsqu'il est perdu ou lorsque l'état spectral est
remis à zéro. Sans nouvel événement, le zéro est ensuite répété à 1 Hz après
deux périodes maximales du motif configuré.

Sur les six captures de banc, les événements reconstruits sont absents des
segments éteints et du motif 2 Hz rejeté. En régime 3 Hz, ils sont espacés de
0,316 à 0,351 s et 95 % se trouvent à moins de 11,7 ms du front descendant RGB
mesuré. Ces résultats proviennent d'un rejeu ; cette synchronisation n'est pas
encore testée sur la cible.

Avec `role.imav.light.adaptive_pattern=true` et `cree_test=false`, le détecteur
spectral continue de fonctionner avec les mêmes paramètres, scores et
événements. Une voie parallèle apprend les triplets et le rythme lent de la
vidéo après trois cycles concordants et qualification rouge/contraste.
Lorsqu'un nouveau motif est acquis, ses flashs observés deviennent la source
de `lit`, sans publication extrapolée dans la pause longue. La perte d'une
voie ne publie pas zéro si l'autre reconnaît toujours un motif. Le délai de
secours vaut 1,8 s lorsque le nouveau motif est acquis ; il conserve sa valeur
standard sinon. Un trou de mesures de 100 ms réinitialise l'apprentissage.
Les réglages standard, y compris `beginning_pattern`, restent effectifs.
Voir [le profil adaptatif](../docs/software/roles/imav_adaptive_light.md)
pour l'activation, le délai d'acquisition et les limites.

### 2.3 Utilisation recommandée par le contrôleur de vol

- utiliser `snr` comme force relative pour la recherche de gradient et la
  triangulation ;
- utiliser `lit` à courte portée pour déclencher le largage du medikit ;
- traiter zéro comme un score expiré et non comme une nouvelle mesure à
  intégrer dans la descente de gradient ;
- appliquer côté contrôleur de vol le lissage spatial et les seuils propres à
  la stratégie de mission ;
- garder les deux modalités indépendantes : l'audio guide l'approche et la
  lumière confirme la zone de largage ;
- ne pas attendre un message combiné : le firmware ne publie volontairement
  que les observations élémentaires `snr` et `lit`.

## 3. Configuration pour l'épreuve

La configuration normale de mission est :

```text
ROLE.imav.beacon = true
role.imav.light.beginning_pattern = false
role.imav.audio.snr_alpha = 1.0
role.imav.debug.publish.optional = false
```

Dans cette configuration, le rôle n'émet que `snr` et `lit` pour la détection
IMAV : une trame par salve sonore et une par flash lumineux reconnus. Après
expiration, chaque score nul est répété à 1 Hz. Le contrôleur de vol ne doit
dépendre d'aucune clé de l'annexe.

`role.imav.time_of_flight` est une fonctionnalité séparée. Si elle est activée,
la distance au sol est publiée avec le message UAVCAN fonctionnel
`uavcan.equipment.range_sensor.Measurement`; cette publication n'est pas
conditionnée par l'option de débogage.

---

# Annexe A — Messages optionnels de mise au point

## A.1 Activation et statut

Les messages de cette annexe sont activés à chaud par :

```text
role.imav.debug.publish.optional = true
```

Ils sont exclusivement destinés aux essais sur banc, aux acquisitions CAN et
au recalage des algorithmes. Ils ne constituent pas une API stable et doivent
rester désactivés pendant l'épreuve IMAV 2026.

## A.2 Diagnostics audio dérivés — 5 Hz

| Clé | Cadence | Unité/domaine | Description de mise au point |
|---|---:|---|---|
| `a0` | 5 Hz | `0..1` | Score spectral instantané du bloc microphone. |
| `p0` | 5 Hz | comptes ADC | Amplitude RMS estimée de la tonalité dominante. |
| `sdb` | 5 Hz | dB | Rapport instantané entre l'énergie de la bande balise et l'énergie globale de la fenêtre. Ce n'est pas le SNR par rapport aux moteurs. |
| `aud` | 5 Hz | `0..1` | Score audio complet, maintenu pendant les silences normaux entre les bips. |
| `frq` | 5 Hz | Hz | Fréquence audio dominante estimée. |
| `cad` | 5 Hz | Hz | Cadence estimée des salves audio. Elle est diagnostique et ne conditionne pas directement `det`. |

## A.3 Diagnostics de la voie optique rapide — 5 Hz

Ces clés décrivent le détecteur lumineux sélectionné, à mémoire finie. Son
score `lfs` est la source directe du message nominal `lit`.

| Clé | Cadence | Unité/domaine | Description de mise au point |
|---|---:|---|---|
| `lrr` | 5 Hz | `0..1` | Fraction rouge de l'excursion RGB positive. |
| `lac` | 5 Hz | rapport | Excursion rouge positive rapportée au fond lumineux. |
| `lis` | 5 Hz | `0..1` | Score instantané d'une impulsion rouge. |
| `lhz` | 5 Hz | Hz | Fréquence du meilleur fondamental dans la fenêtre rapide. |
| `lcs` | 5 Hz | `0..1` | Cohérence du fondamental rapide dans le contraste rouge. |
| `lfs` | 5 Hz | `0..1` | Score final de la fenêtre rapide : cohérence, couleur, rapport H2/H1 et phase relative. |
| `lps` | 5 Hz | `0..1` | Amplitude du fondamental rapide rapportée au niveau rouge moyen. |
| `lpc` | 5 Hz | compteur | Nombre cumulé d'épisodes où la voie rapide s'est verrouillée depuis le démarrage. |
| `lsa` | 5 Hz | compteur | Nombre cumulé d'épisodes de surcharge OPT4060. |
| `ler` | 5 Hz | compteur | Nombre cumulé d'erreurs de lecture OPT4060. |
| `lgp` | 5 Hz | compteur | Nombre cumulé de trous détectés dans l'acquisition lumineuse. |
| `lon` | 5 Hz | ms | Durée haute estimée dans la fenêtre rapide à partir de H2/H1. |
| `lof` | 5 Hz | ms | Durée basse estimée dans la fenêtre rapide à partir de H2/H1. |
| `lts` | 5 Hz | `0..1` | Ressemblance harmonique de la fenêtre rapide au créneau asymétrique attendu. |
| `lpt` | 5 Hz | entier | Motif ayant fourni le score courant : `0` aucun, `1` démarrage, `2` régime établi, `3` CREE, `4` adaptatif. |
| `lpn` | 5 Hz | entier | Nombre de flashs par cycle appris en adaptatif : `1` ou `3`, sinon `0`. |

Lorsque `lpt=4`, `lhz` est la fréquence du cycle complet (environ 0,70 Hz pour
les triplets), `lon/lof` les dernières durées observées, `lcs/lts` la cohérence
temporelle et `lfs/lps` le score appris. Les descriptions spectrales de ces
clés dans le tableau s'appliquent aux profils standard/CREE.

Les valeurs nominales sont `high_ms=100`, `steady_low_ms=233` et
`beginning_low_ms=400`. `role.imav.light.beginning_pattern=false` ne construit
et n'évalue que la banque du régime établi ; sa valeur par défaut évite qu'un
leurre à environ 2 Hz soit accepté pendant l'épreuve. La valeur `true`, utile
sur le banc lorsque la balise est allumée pendant l'acquisition, autorise les
deux banques et retient la meilleure. Ces quatre paramètres sont lus au
démarrage du rôle : il faut redémarrer après modification.

## A.4 Échantillons optiques bruts — jusqu'à environ 139 Hz

Ces sept clés sont envoyées pour chaque échantillon OPT4060. À 1,8 ms par voie,
l'interruption data-ready signale un groupe RGBW toutes les 7,2 ms, soit au
plus environ 139 groupes/s hors reprises automatiques de plage. Lorsque le
VL53L4CX est activé, l'acquisition optique est suspendue pendant chaque mesure
de distance.

| Clé | Cadence | Unité/domaine | Description de mise au point |
|---|---:|---|---|
| `lrd` | jusqu'à ~139 Hz | code ADC linéarisé | Canal rouge brut OPT4060. |
| `lgn` | jusqu'à ~139 Hz | code ADC linéarisé | Canal vert brut OPT4060. |
| `lbl` | jusqu'à ~139 Hz | code ADC linéarisé | Canal bleu brut OPT4060. |
| `lwh` | jusqu'à ~139 Hz | code ADC linéarisé | Canal large bande brut OPT4060. |
| `lov` | jusqu'à ~139 Hz | `0` ou `1` | Indique que l'échantillon est en surcharge. |
| `lct` | jusqu'à ~139 Hz | compteur | Compteur d'échantillons permettant de reconstruire les groupes et de détecter les pertes. |
| `ltu` | jusqu'à ~139 Hz | microsecondes modulo 2^24 | Horodatage MCU associé à l'échantillon. La représentation reste exacte dans le flottant UAVCAN et reboucle toutes les 16,777216 s. |

Les quatre codes RGBW ont au plus 20 bits significatifs après application de
l'exposant OPT4060 ; ils restent donc représentables exactement malgré leur
étendue linéarisée. Pour le post-traitement, il faut regrouper les valeurs avec
`lct`, dérouler `ltu` et accepter qu'une capture commence ou se termine au
milieu d'un groupe.

## A.5 Diagnostics du télémètre — 5 Hz

Ces deux clés n'existent que si le débogage optionnel et
`role.imav.time_of_flight` sont tous deux activés :

| Clé | Cadence | Unité/domaine | Description de mise au point |
|---|---:|---|---|
| `rng` | 5 Hz | m | Dernière distance VL53L4CX valide ; `-1` si la dernière mesure n'est pas valide. |
| `rsg` | 5 Hz | kcps/SPAD | Force du signal reçu par le VL53L4CX. |

Les clés `rng/rsg` répètent la dernière mesure à la cadence de télémétrie de
5 Hz. La cadence d'acquisition fonctionnelle du télémètre est réglée par
`role.imav.tof.period_ms`, entre 100 et 1 000 ms, avec 200 ms par défaut. Le
message standard `uavcan.equipment.range_sensor.Measurement` est envoyé à
chaque acquisition et reste indépendant des clés de débogage `rng/rsg`.

## A.6 Charge CAN du mode de mise au point

Avec le débogage activé, sans télémètre et avec les deux détections en régime
établi à 3 Hz :

- environ 3 trames `snr` et 3 trames `lit` par seconde lorsque la balise est
  reçue, ou deux trames nulles par seconde lorsqu'elle ne l'est pas ;
- 105 trames/s pour les 21 diagnostics dérivés à 5 Hz ;
- jusqu'à environ 973 trames/s pour les sept clés brutes à 139 Hz ;
- total maximal voisin de 1 084 trames/s.

Le télémètre ajoute dix trames `rng/rsg` par seconde,
soit environ 1 099 trames/s, en plus de son message UAVCAN standard. Cette charge
reste techniquement supportable sur le banc à 1 Mbit/s, mais elle consomme
environ 15 % du réseau et noie les outils avec des données inutiles pour
la mission. C'est la raison principale, en plus de la stabilité de l'interface,
pour laquelle `role.imav.debug.publish.optional` doit rester à `false` pendant
l'épreuve.

## A.7 Outils Linux de mise au point

L'enregistreur exhaustif se trouve dans `tools/imav_monitor` :

```sh
python3 tools/imav_monitor/record_imav_can.py \
  --interface can0 --source-node 10 --duration 60 \
  --output /tmp/imav_capture.csv
```

Il utilise uniquement `can0` et les messages UAVCAN ; aucune donnée de la
liaison série de débogage n'entre dans la détection ou l'enregistrement.
