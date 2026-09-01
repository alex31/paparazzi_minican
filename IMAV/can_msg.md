# Messages CAN du rôle IMAV 2026

## Objet et périmètre

Le rôle `ROLE.imav.beacon` publie trois grandeurs destinées au contrôleur de
vol : détection sonore, force sonore relative et score de détection lumineuse.
Elles constituent l'interface CAN opérationnelle du rôle pour l'épreuve IMAV
2026.

Toutes les autres clés décrites en annexe sont des données d'instrumentation.
Elles servent à enregistrer les capteurs, mettre au point les algorithmes et
expliquer une détection ou un faux positif. **Elles ne doivent pas être
consommées par la logique de mission pendant l'épreuve IMAV 2026.** Leur
format, leur cadence et même leur présence peuvent évoluer pendant la mise au
point.

## 1. Transport de l'interface opérationnelle

Les trois valeurs sont diffusées individuellement sous forme de messages
UAVCAN v0 `uavcan.protocol.debug.KeyValue` (Data Type ID 16370), avec une
priorité CAN basse :

- réseau CAN classique à 1 Mbit/s, sans CAN FD ;
- clé ASCII de trois caractères ;
- valeur `float32` ;
- cadence nominale de 5 Hz par clé, soit 15 trames/s au total ;
- publication broadcast : aucun acquittement ni abonnement préalable ;
- le Node-ID UAVCAN source identifie la MicroCAN qui porte les capteurs.

La période demandée par le firmware est de 200 ms. La publication est
déclenchée depuis le traitement audio par blocs ; les intervalles observés
peuvent donc alterner légèrement autour de 200 ms. Il faut interpréter la
cadence comme une moyenne de 5 Hz, et non comme une horloge de synchronisation.

Chaque clé est un transfert UAVCAN indépendant. Les trois valeurs ne forment
pas un paquet atomique et le message ne contient pas l'instant de mesure. Le
récepteur doit mémoriser la dernière valeur de chaque couple `(Node-ID, clé)`,
utiliser son heure locale de réception et invalider les données si elles ne
sont plus renouvelées.

## 2. Messages systématiques à 5 Hz

| Clé | Unité/domaine | Signification opérationnelle |
|---|---|---|
| `det` | exactement `0.0` ou `1.0` | État de détection audio après filtrage spectral, cohérence de blocs et hystérésis. `1.0` signifie que la signature sonore de la balise est actuellement reconnue. |
| `snr` | dB relatifs | Force de la dernière salve sonore reconnue au-dessus du plancher de bruit adaptatif dans la bande 2–3 kHz. La valeur est maintenue entre les bips, puis revient vers zéro lorsque la mesure devient périmée. |
| `lit` | score continu de `0.0` à `1.0` | Confiance dans une modulation lumineuse rouge proche de 3 Hz. Le score provient d'une DFT glissante, normalisée par le bruit spectral optique local et qualifiée par la cohérence et la couleur périodique. |

### 2.1 Interprétation de `det`

`det` est la seule grandeur nominale déjà discrétisée. Elle indique la
présence de la signature audio ; elle ne mesure ni une pression acoustique en
dB SPL, ni une distance.

Le bruit blanc continu des moteurs participe à l'apprentissage du plancher de
bruit. Une hausse large bande ne suffit pas à valider `det` : il faut une
concentration spectrale compatible avec la balise.

### 2.2 Interprétation de `snr`

`snr` est la grandeur principale pour comparer la force sonore en différents
points de la trajectoire et diriger le drone vers la balise. Elle est relative
au bruit moteur appris localement. Elle convient donc à une cartographie
spatiale ou à une recherche de gradient avec la même MicroCAN.

Ce n'est pas une mesure acoustique absolue. Il ne faut pas comparer directement
deux MicroCAN sans calibration, ni convertir `snr` en distance avec une loi
universelle. L'orientation du microphone, le vent, le régime moteur et les
réflexions acoustiques modifient la valeur.

### 2.3 Interprétation de `lit`

`lit` est également un score continu, pas un booléen et pas une mesure de lux.
Le fond lumineux continu est retiré. Le détecteur recherche un pic entre 2,8
et 3,2 Hz et l'évalue par rapport aux fréquences voisines ; il reste ainsi
utilisable lorsque le signal est faible devant la lumière réfléchie par le
sol.

L'intégration spectrale est volontairement lente, avec une mémoire de l'ordre
de dix secondes, mais une balise forte peut faire monter le score beaucoup
plus vite. Le firmware force `lit` à zéro si aucun nouvel échantillon OPT4060
n'a été reçu depuis 200 ms.

### 2.4 Utilisation recommandée par le contrôleur de vol

- utiliser `det` comme preuve discrète de présence sonore ;
- utiliser `snr` comme force relative pour la recherche de gradient et la
  triangulation ;
- utiliser `lit` comme preuve lumineuse continue et comme confirmation
  indépendante ;
- appliquer côté contrôleur de vol les temporisations, lissage spatial et
  seuils propres à la stratégie de mission ;
- ne pas imposer une condition rigide `det ET lit` : les deux modalités sont
  indépendantes et l'une peut être momentanément masquée ou défaillante ;
- ne pas attendre un message combiné : le firmware ne publie volontairement
  que les observations élémentaires `det`, `snr` et `lit`.

## 3. Configuration pour l'épreuve

La configuration normale de mission est :

```text
ROLE.imav.beacon = true
role.imav.debug.publish.optional = false
```

Dans cette configuration, le rôle n'émet que `det`, `snr` et `lit` pour la
détection IMAV, soit 15 trames/s. Le contrôleur de vol ne doit dépendre d'aucune
clé de l'annexe.

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

## A.3 Diagnostics optiques historiques — 5 Hz

Ces clés décrivent l'ancien détecteur temporel par seuils et fronts. Elles
restent utiles pour comparer les algorithmes, mais elles ne produisent plus le
score nominal `lit`.

| Clé | Cadence | Unité/domaine | Description de mise au point |
|---|---:|---|---|
| `lrr` | 5 Hz | `0..1` | Fraction rouge de l'excursion RGB positive. |
| `lac` | 5 Hz | rapport | Excursion rouge positive rapportée au fond lumineux. |
| `lis` | 5 Hz | `0..1` | Score instantané d'une impulsion rouge. |
| `lhz` | 5 Hz | Hz | Cadence mesurée entre les fronts lumineux acceptés. |
| `lcs` | 5 Hz | `0..1` | Confiance de l'ancien détecteur dans une cadence proche de 2 ou 3 Hz. |
| `lps` | 5 Hz | `0..1` | Force du flash courant ou du dernier flash accepté. |
| `lpc` | 5 Hz | compteur | Nombre cumulé de fronts lumineux acceptés depuis le démarrage. |
| `lsa` | 5 Hz | compteur | Nombre cumulé d'épisodes de surcharge OPT4060. |
| `ler` | 5 Hz | compteur | Nombre cumulé d'erreurs de lecture OPT4060. |
| `lgp` | 5 Hz | compteur | Nombre cumulé de trous détectés dans l'acquisition lumineuse. |

## A.4 Diagnostics du détecteur spectral optique — 5 Hz

| Clé | Cadence | Unité/domaine | Description de mise au point |
|---|---:|---|---|
| `lsc` | 5 Hz | `0..1` | Score spectral interne utilisé pour produire `lit`, avant le contrôle de fraîcheur effectué au moment de la publication nominale. |
| `lsn` | 5 Hz | dB | Proéminence du meilleur bin entre 2,8 et 3,2 Hz par rapport à la médiane du bruit spectral local. |
| `lco` | 5 Hz | `0..1` | Cohérence du fondamental périodique dans le contraste rouge. |
| `lrf` | 5 Hz | `0..1` | Fraction rouge de la composante RGB périodique complexe. |
| `lfq` | 5 Hz | Hz | Fréquence du meilleur bin spectral, entre 2,8 et 3,2 Hz. |
| `lhr` | 5 Hz | rapport d'amplitudes | Rapport entre la deuxième harmonique à 6 Hz et le fondamental sélectionné. La deuxième harmonique est observée mais n'est pas obligatoire pour valider `lit`. |

## A.5 Échantillons optiques bruts — environ 100 Hz

Ces sept clés sont envoyées pour chaque échantillon OPT4060. Sans télémètre,
le débit mesuré est proche de 99 à 100 groupes/s. Lorsque le VL53L4CX est
activé, l'acquisition optique est suspendue pendant chaque mesure de distance.

| Clé | Cadence | Unité/domaine | Description de mise au point |
|---|---:|---|---|
| `lrd` | ~100 Hz | code ADC linéarisé | Canal rouge brut OPT4060. |
| `lgn` | ~100 Hz | code ADC linéarisé | Canal vert brut OPT4060. |
| `lbl` | ~100 Hz | code ADC linéarisé | Canal bleu brut OPT4060. |
| `lwh` | ~100 Hz | code ADC linéarisé | Canal large bande brut OPT4060. |
| `lov` | ~100 Hz | `0` ou `1` | Indique que l'échantillon est en surcharge. |
| `lct` | ~100 Hz | compteur | Compteur d'échantillons permettant de reconstruire les groupes et de détecter les pertes. |
| `ltu` | ~100 Hz | microsecondes modulo 2^24 | Horodatage MCU associé à l'échantillon. La représentation reste exacte dans le flottant UAVCAN et reboucle toutes les 16,777216 s. |

Les quatre codes RGBW ont au plus 20 bits significatifs après application de
l'exposant OPT4060 ; ils restent donc représentables exactement malgré leur
étendue linéarisée. Pour le post-traitement, il faut regrouper les valeurs avec
`lct`, dérouler `ltu` et accepter qu'une capture commence ou se termine au
milieu d'un groupe.

## A.6 Diagnostics du télémètre — 5 Hz

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

## A.7 Charge CAN du mode de mise au point

Avec le débogage activé et sans télémètre :

- 15 trames/s nominales pour `det`, `snr` et `lit` ;
- 110 trames/s pour les 22 diagnostics dérivés à 5 Hz ;
- environ 700 trames/s pour les sept clés brutes à 100 Hz ;
- total voisin de 825 trames/s.

Le télémètre ajoute dix trames `rng/rsg` par seconde,
soit environ 835 trames/s, en plus de son message UAVCAN standard. Cette charge
reste techniquement supportable sur le banc à 1 Mbit/s, mais elle consomme
environ 10 à 12 % du réseau et noie les outils avec des données inutiles pour
la mission. C'est la raison principale, en plus de la stabilité de l'interface,
pour laquelle `role.imav.debug.publish.optional` doit rester à `false` pendant
l'épreuve.

## A.8 Outils Linux de mise au point

L'enregistreur exhaustif se trouve dans `tools/imav_monitor` :

```sh
python3 tools/imav_monitor/record_imav_can.py \
  --interface can0 --source-node 10 --duration 60 \
  --output /tmp/imav_capture.csv
```

Il utilise uniquement `can0` et les messages UAVCAN ; aucune donnée de la
liaison série de débogage n'entre dans la détection ou l'enregistrement.
