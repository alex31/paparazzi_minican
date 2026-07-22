# Détection IMAV 2026 du mannequin équipé d'un MSA motionSCOUT

> Note de conception et de reprise de contexte — 17 juillet 2026  
> Projet : MicroCAN v5, branche `imav2026`  
> MCU : STM32G491KEU6, ChibiOS 21.11, DroneCAN/UAVCAN v0  
> Statut : étude d'architecture ; aucune valeur de seuil ne doit être figée avant la mesure d'une balise réelle.

Convention de lecture :

- **règlement** : exigence explicitement écrite dans le PDF IMAV ;
- **constructeur** : donnée publiée par MSA ou un fabricant de composants ;
- **calcul idéal** : ordre de grandeur sans bruit, directivité ni réflexion ;
- **hypothèse de conception** : choix provisoire à tester ;
- **à mesurer** : information inconnue qui ne doit pas être transformée en spécification.

## 1. Objet de ce document

Cette note rassemble les conclusions techniques nécessaires pour reprendre le travail sur une autre machine sans avoir à refaire l'étude :

- contraintes extraites du règlement IMAV 2026 ;
- comportement connu de la balise MSA motionSCOUT K-T-R ;
- principe de recherche à basse altitude par cartographie de la force du signal ;
- choix des microphones et des capteurs optiques ;
- adaptation réaliste à la carte MicroCAN v5 existante ;
- acquisition audio en flux continu avec seulement quelques kilo-octets de RAM ;
- traitement numérique, fusion son/lumière et interface DroneCAN ;
- montage suspendu sous le drone ;
- plan de prototypage et d'essais ;
- risques, inconnues et décisions encore ouvertes.

La première conclusion est simple : **la MicroCAN v5 et ses 112 Kio de RAM suffisent très largement pour reconnaître l'alarme**, car l'audio n'est jamais enregistré intégralement. Un DMA circulaire contient seulement 20 à 40 ms de signal, le bloc est traité, puis immédiatement écrasé par le bloc suivant.

La deuxième conclusion est liée au matériel existant et aux décisions prises après l'étude initiale : **le MVP utilise un seul microphone analogique IM68A130(A), directement relié à PA3/ADC1_IN4, un OPT4060 orienté vers le sol pour les flashs et un VL53L4CX pour la hauteur du module**. PA4 est de nouveau libre. L'OPT4060 fournit les quatre voies RGBW à environ 100 Hz, sans FIFO ; sa mesure est explicitement arrêtée pendant chaque mesure ponctuelle du télémètre 940 nm.

Les décisions suivantes font foi lorsqu'une section historique du document semble encore présenter une variante antérieure :

- un microphone unique sur PA3 = ADC1_IN4 ;
- PA4 libre pour une autre fonction ;
- PA2 conservée pour `DBG_TX` ;
- pas d'OPAMP interne ni externe dans le premier prototype ;
- liaison continue en tension entre la sortie déjà polarisée de l'IM68A130(A) et l'ADC, avec seulement découplage et petit réseau RC passif ;
- suréchantillonnage matériel ADC envisagé en x4 ou x16, sans augmentation de la taille des tampons DMA ;
- un seul OPT4060 sous le drone, sans sectorisation optique ;
- VL53L4CX piloté par le composant officiel ST provenant du sous-module `STMicroelectronics/x-cube-tof1` ;
- pas de canaux injectés : un seul thread arbitre explicitement audio et mesures lentes ;
- acquisition lumière et mesure ToF strictement exclusives.

## 2. Exigences du règlement IMAV 2026

Source locale : [Rulebook_IMAV2026_V2.1.pdf](Rulebook_IMAV2026_V2.1.pdf), sections 5.4.7 et 5.4.8, page 34.

La mission extérieure 4 consiste à déposer ou larguer un kit de premiers secours près d'un mannequin équipé d'un dispositif d'homme mort.

Contraintes directement utiles :

- dispositif annoncé : **MSA motionSCOUT K-T-R** ;
- fréquence sonore annoncée : **2,6 à 3,0 kHz** ;
- niveau annoncé : **95 dB à 3 m** ;
- signal lumineux : **deux LED « ultra bright »** ;
- le point GPS central de la zone de recherche est fourni le jour de l'épreuve ;
- trois mannequins se trouvent dans un rayon de 25 m autour de ce point ;
- un seul mannequin porte la balise ;
- le colis doit être déposé ou largué à une altitude strictement inférieure à 2 m ;
- la composante principale du score est doublée si le bon mannequin est choisi ;
- la distance de notation est mesurée entre le nombril du mannequin et le colis ;
- en dessous de 50 cm, la distance prise dans la formule reste plafonnée à 50 cm ;
- la composante de mission devient nulle à 300 cm ;
- les traitements entièrement embarqués permettent de conserver le facteur d'autonomie `A = 1` ;
- un calcul sur PC au sol réduit ce facteur à 0,7 ; une intervention manuelle le réduit à 0,4.

Le règlement n'impose ni type de capteur ni bus particulier. ADC, I2C, SPI et CAN sont donc libres. Il n'impose pas non plus que le son et la lumière soient détectés simultanément.

### 2.1 Ambiguïtés réglementaires à faire confirmer

Le texte contient à la fois « sound of a certain power (to be defined) » et la valeur « 95 dB at 3 m ». Il subsiste donc une contradiction sur le caractère définitif du niveau sonore.

Le règlement ne précise pas :

- la longueur d'onde ou la couleur exacte des LED ;
- leur intensité en candela ;
- la durée des impulsions et leur cadence ;
- l'angle d'émission et l'orientation de la balise sur le mannequin ;
- la synchronisation éventuelle entre son et lumière ;
- la forme exacte du son, ses harmoniques et ses tolérances ;
- la pondération du niveau sonore, sa tolérance et les conditions de mesure ;
- les conditions de vent, de soleil, d'occultation et de bruit ambiant ;
- la distance minimale à laquelle le système doit garantir la détection.

Il faut demander aux organisateurs, dans l'ordre de préférence :

1. le prêt d'un exemplaire identique à celui de l'épreuve ;
2. à défaut, un fichier WAV non compressé et une vidéo à exposition fixe ;
3. la référence exacte, la date de fabrication, la couleur et la cadence des LED ;
4. la confirmation écrite du niveau sonore retenu.

## 3. Informations complémentaires sur le MSA motionSCOUT

La [fiche produit MSA motionSCOUT](https://s7d9.scene7.com/is/content/minesafetyappliances/motionSCOUT%20Bulletin%20-%20ES) indique :

- fréquence : 2,6 à 3,0 kHz ;
- alarme : 95 dB à 3 m ;
- préalarme : deux signaux par seconde, à niveau réduit ;
- alarme complète : trois signaux par seconde, à niveau élevé ;
- deux LED rouges d'alarme très lumineuses ;
- voyant d'état bicolore séparé ;
- référence du modèle K-T-R : `10088478` ;
- étanchéité IP67.

Les cadences proches de 2 Hz en préalarme et de 3 Hz en alarme complète sont
des informations distinctives, mais les deux états doivent être considérés
valides. La décision audio repose donc d'abord sur l'énergie concentrée dans la
bande utile ; la cadence reste une mesure de diagnostic et ne bloque pas une
détection.

Une analyse de la
[seconde vidéo de démonstration](https://www.youtube.com/watch?v=fVBscz2agI4)
observe un maximum spectral vers 2 433 Hz, un chirp rapide d'environ 2,1 à
2,5 kHz à l'intérieur d'un bip et une période de préalarme de 0,51 s. Cette
observation non normative justifie la bande par défaut 2,0–3,0 kHz et le cumul
d'énergie sur toute la bande plutôt que la recherche d'une sinusoïde fixe.

### 3.1 Avis de service MSA de février 2026

Un [avis de service officiel MSA](https://assetlibrary.msasafety.com/m/1daafc2b15a7c2a5/original/Avis-de-service-Dispositif-MSA-motionSCOUT-PASS-Fevrier-2026.pdf) concerne notamment les unités `10088478` fabriquées entre juin 2024 et décembre 2025. Sur les unités concernées, l'alarme sonore peut cesser de fonctionner alors que l'alarme lumineuse continue.

Conséquences de conception :

- ne jamais valider la détection uniquement par un `son ET lumière` rigide ;
- accepter un signal sonore seul s'il est très caractéristique et persistant ;
- accepter un signal lumineux seul avec un seuil et une durée de confirmation plus exigeants ;
- augmenter fortement la confiance lorsque les deux modalités concordent ;
- demander aux organisateurs de confirmer que leur exemplaire n'est pas concerné ou a été remplacé.

## 4. Concept opérationnel retenu

Le drone survole la zone à environ 1,5 m. La MicroCAN et ses capteurs sont suspendus sous le drone par un câble et maintenus verticalement par un petit poids. La hauteur réelle du capteur au-dessus du sol sera donc inférieure à l'altitude du centre du drone.

Le capteur n'a pas besoin de donner instantanément un azimut. Il fournit une **mesure de confiance et de force de signal horodatée**. Le contrôleur de vol associe chaque mesure à la position du capteur et construit une carte spatiale. Le maximum de cette carte donne une zone candidate, ensuite raffinée par une petite croix ou une spirale.

Deux stratégies de recherche sont possibles :

1. si une caméra détecte déjà les trois mannequins, les survoler successivement et comparer les scores pendant un stationnaire de 1 à 2 s ;
2. sinon, réaliser une grille grossière sur la zone, puis raffiner autour du maximum.

La première stratégie est beaucoup plus rapide et devrait être privilégiée si la détection visuelle des mannequins est déjà disponible.

## 5. Faisabilité physique à basse altitude

### 5.1 Niveau acoustique théorique

En champ libre idéal, en supposant 95 dB SPL à 3 m :

\[
L(r)=95-20\log_{10}(r/3)
\]

Si le capteur lui-même est à 1,5 m exactement au-dessus de la balise :

\[
L(1{,}5)=95+20\log_{10}(2)\approx101\ \mathrm{dB\ SPL}
\]

Si la suspension descend le capteur à 1 m au-dessus de la balise, le niveau idéal atteint environ 104,5 dB SPL.

Pour une hauteur capteur de 1,5 m, la variation idéale avec le décalage horizontal est :

| Décalage horizontal | Distance capteur-balise | Baisse par rapport au passage vertical |
|---:|---:|---:|
| 0 m | 1,50 m | 0 dB |
| 1 m | 1,80 m | -1,6 dB |
| 2 m | 2,50 m | -4,4 dB |
| 3 m | 3,35 m | -7,0 dB |
| 5 m | 5,22 m | -10,8 dB |

Le pic spatial devrait donc être suffisamment marqué pour une cartographie.

### 5.2 Limites du modèle

Ces calculs sont seulement un ordre de grandeur. Le niveau réel sera modifié par :

- la directivité du buzzer ;
- le mannequin, ses vêtements et la position de la balise ;
- les réflexions sur le sol et le corps ;
- les interférences cohérentes dues à une fréquence proche de 2,8 kHz ;
- le bruit aérodynamique sur le microphone ;
- les hélices et les harmoniques des moteurs ;
- les autres drones, véhicules et alarmes.

Il ne faut donc pas ajuster aveuglément une loi parfaite en `1/r`. Une carte lissée et un maximum robuste sont préférables.

### 5.3 Lumière

Pour une source ponctuelle idéale, l'éclairement décroît en `1/r²`. À 1 à 1,5 m, les LED devraient être beaucoup plus faciles à détecter qu'à 25 m. La principale difficulté devient la dynamique entre le flash et le fond solaire, ainsi que l'orientation des LED.

Les valeurs optiques ne peuvent pas être calculées sérieusement tant que l'intensité, la longueur d'onde, le diagramme d'émission et la durée d'impulsion ne sont pas connus.

## 6. Matériel MicroCAN v5 existant

Références locales :

- [schéma MicroCAN v5](../HARDWARE/MICROCAN/MicroCan_v5_schematic.pdf) ;
- [description de carte](../docs/hardware/boards/microcan_v5.md) ;
- [carte des broches](../microcan/cfg/MICROCAN.cfg) ;
- [configuration ChibiOS](../microcan/cfg/mcuconf.h) ;
- [script de l'éditeur de liens](../microcan/cfg/STM32G491xE.ld).

La carte contient déjà :

- STM32G491KEU6 à 170 MHz avec FPU simple précision ;
- 512 Kio de Flash et 112 Kio de RAM ;
- transceiver CAN 3,3 V TCAN332 ;
- terminaison CAN 120 ohms commandable par logiciel ;
- connecteurs CAN entrée/sortie à quatre fils ;
- alimentation 6 à 26 V, buck 5 V puis LDO 3,3 V ;
- connecteur I2C avec GND, 5 V, 3,3 V, SDA et SCL ;
- résistances de tirage I2C activables par logiciel ;
- connecteur SPI partagé avec la mémoire M95P interne ;
- connecteurs UART, PWM et sonde de débogage.

Il n'est donc pas nécessaire d'ajouter un transceiver CAN ou une alimentation générale sur la carte capteur. Les recommandations génériques de type TCAN3413, buck externe et terminaison séparée ne s'appliquent pas à cette MicroCAN déjà réalisée.

### 6.1 Broches analogiques réellement accessibles

Le boîtier 32 broches ne fournit pas un connecteur ADC dédié.

En considérant toutes les broches du MCU effectivement exposées sur les connecteurs de la MicroCAN, et en conservant I2C1, SPI1/M95P et CAN, la paire retenue est :

| Broche MCU | Connecteur/fonction actuelle | Fonction audio retenue | Décision |
|---|---|---|---|
| PA2 | sonde J3, `DBG_TX` | `ADC1_IN3` possible | conservée pour une trace TX minimale |
| PA3 | sonde J3, `DBG_RX` | `ADC1_IN4`, microphone 1 | retenue ; perte du RX de débogage |
| PA4 | `SPI_PERIPH_CS` | `ADC2_IN17`, microphone 2 | retenue ; perte du CS SPI externe uniquement |

PA3 et PA4 appartiennent à deux ADC distincts. Ils peuvent donc convertir en parallèle sur le même front de timer, chacun avec une séquence régulière d'un seul canal et son propre DMA. C'est préférable à deux canaux séquentiels du même ADC pour conserver une bonne cohérence temporelle entre microphones.

Les autres entrées analogiques ne sont pas retenues :

| Broche MCU | Fonction ADC | Routage actuel | Motif |
|---|---|---|---|
| PA0 | `ADC1/2_IN1` | mesure VIN | conservée pour la surveillance d'alimentation |
| PA1 | `ADC1/2_IN2` | commande des pull-up I2C | ne pas perturber I2C1 |
| PA5 | `ADC2_IN13` | `SPI1_SCK` | conserver SPI1/M95P |
| PA6 | `ADC2_IN3` | `SPI1_MISO` | conserver SPI1/M95P |
| PA7 | `ADC2_IN4` | `SPI1_MOSI` | conserver SPI1/M95P |
| PB0 | `ADC1_IN15` | chip-select M95P interne | conserver la mémoire |
| PF0/PF1 | fonctions analogiques possibles | oscillateur/terminaison CAN | fonctions de carte prioritaires |

Les deux réseaux microphone-vers-ADC doivent être identiques et les voies devront être calibrées en gain et offset. L'écart d'amplitude entre les microphones ne doit pas être interprété directement comme une direction avant cette calibration.

### 6.2 ADC et timers dans le firmware actuel

Le firmware actuel :

- active ADC1 pour la tension batterie, la température interne et VREFINT ;
- laisse ADC2 désactivé ;
- utilise ADC1 en conversion circulaire ;
- utilise TIM2 pour l'OS ;
- utilise potentiellement TIM7 pour DShot ;
- utilise FDCAN2 pour le réseau ;
- active I2C1 sur PA15/PB7 ;
- dispose d'un gestionnaire de ressources pour les rôles.

`Adc::start()` est actuellement appelé inconditionnellement dans `main.cpp`. ADC1 exécute une conversion circulaire de PA0/VIN, température interne et VREFINT. Cette organisation doit être refactorée, car ADC1 devient aussi la voie audio de PA3.

La solution retenue reste simple et n'utilise pas les groupes injectés : **un thread unique possède ADC1 et ADC2 et sérialise toutes les opérations de contrôle**.

Séquence de fonctionnement :

1. configurer ADC1 avec un groupe audio régulier d'un canal PA3 et ADC2 avec un groupe audio régulier d'un canal PA4 ;
2. armer les deux DMA circulaires puis les déclencher avec le même timer à 24 kéch/s ;
3. traiter les demi-buffers pendant que les DMA remplissent les autres moitiés ;
4. à intervalle lent, arrêter proprement le trigger et les deux acquisitions audio à une frontière de bloc ;
5. désactiver/réactiver l'ADC concerné ou effectuer une conversion factice, puis utiliser sur ADC1 un autre `ADCConversionGroup` pour l'acquisition one-shot VIN/température/VREF ;
6. restaurer les deux groupes audio, relancer le timer et jeter le premier résultat de chaque ADC ;
7. signaler le trou d'acquisition au DSP au lieu de recoller artificiellement les blocs.

Une perte occasionnelle de quelques échantillons est acceptable pour cette détection par salves. Cette architecture garantit qu'ADC1 n'est jamais utilisé simultanément par la surveillance lente et l'audio, sans complexité de conversions injectées.

Modifications attendues :

- passer `STM32_ADC_USE_ADC2` à `TRUE` ;
- conserver `STM32_ADC_DUAL_MODE` à `FALSE` : ce sont deux drivers et deux DMA indépendants ;
- ajouter ADC1, ADC2 et le timer au gestionnaire de ressources du rôle ;
- empêcher `adcSurvey.cpp` de lancer sa propre conversion concurrente ;
- conserver I2C1 pour l'OPT4048 et éventuellement l'IMU.

Le choix exact du timer doit être vérifié lors de l'implémentation avec tous les rôles qui seront activés sur le drone. TIM7 est déjà activé et utilisé par DShot. TIM2 est réservé à l'OS d'après le schéma. **TIM6, actuellement désactivé, est le premier candidat à vérifier** pour générer un TRGO à 24 kHz. Il ne faut pas figer ce choix avant d'avoir contrôlé la matrice de déclenchement ADC2 et la configuration de mission.

Le fichier CubeMX `.ioc` est partiellement obsolète : il indique notamment 160 MHz, tandis que le firmware vise 170 MHz. Les fichiers ChibiOS, le schéma et le code source sont les références à privilégier.

## 7. Architecture recommandée par étapes

### 7.1 MVP compatible avec la MicroCAN existante

```text
IM68A130(A) n°1 -> RC passif -> PA3 / ADC1_IN4 -> DMA 1 --+
                                                               |
IM68A130(A) n°2 -> RC passif -> PA4 / ADC2_IN17 -> DMA 2 --+--> STM32G491
                                                               |    -> score horodaté -> DroneCAN
OPT4048 couleur -> I2C1

IMU 6 axes optionnelle -> I2C1
```

Cette architecture apporte déjà :

- reconnaissance spectrale et temporelle du son sur deux voies synchrones ;
- mesure de la force du signal sonore ;
- comparaison haut/bas et rejet partiel du bruit venant des hélices ;
- détection indépendante des flashs rouges ;
- fusion des deux scores ;
- cartographie sur le contrôleur de vol ;
- aucune modification de la carte MicroCAN elle-même, mais utilisation de PA3 sur J3 et PA4 sur J6 ;
- aucune perte de l'accès à la mémoire M95P.

À basse altitude, la cartographie d'amplitude resterait possible avec un seul microphone. Les deux voies sont néanmoins retenues parce que les broches et les deux ADC sont disponibles et que la comparaison verticale peut aider face au bruit propre du drone.

### 7.2 Paire verticale retenue

Deux microphones espacés verticalement de 40 à 50 mm permettent d'exploiter le sens de propagation :

- le son venant du sol atteint d'abord le microphone inférieur ;
- le son direct des hélices atteint d'abord le microphone supérieur.

À 2,8 kHz, la longueur d'onde vaut environ 12,2 cm. Avec 40 mm d'écart, le déphasage idéal pour une onde verticale est voisin de 117 degrés, avec un signe opposé selon que le son vient du haut ou du bas.

Cette comparaison n'exige finalement pas d'ADC externe : PA3 alimente ADC1 et PA4 alimente ADC2. Les deux groupes réguliers sont déclenchés par le même événement de timer. Les deux callbacks DMA ne sont pas nécessairement appelés dans un ordre déterministe ; le thread DSP ne traite une demi-fenêtre stéréo qu'après réception des deux indicateurs correspondant au même numéro de bloc.

L'espacement, le signe de phase et la réponse des deux cavités acoustiques devront être mesurés. Une simple différence de niveau n'est pas suffisante à elle seule : on combine niveau, cohérence dans la bande 2,6–3,0 kHz et éventuellement phase complexe du Goertzel.

### 7.3 Amélioration optique analogique

Une photodiode analogique avec TIA donne davantage de contrôle sous le soleil qu'un capteur de lumière intégré : gain continu faible, suppression du fond, amplification AC et détection de saturation.

Sur la carte actuelle, PA4 sera déjà utilisé par le microphone. Les solutions sont alors :

- ADC I2C rapide/multicanal pour les photodiodes ;
- ADC SPI externe commun à une carte fille son/lumière ;
- OPT4048 seul pour le MVP, puis décision à partir des essais.

À 1 à 1,5 m, l'OPT4048 a de bonnes chances de suffire ; il est donc inutile de complexifier la première carte fille avant les mesures.

## 8. Choix des capteurs audio

### 8.1 Choix MVP : IM68A130 ou IM68A130A

[Infineon IM68A130](https://www.infineon.com/part/IM68A130)

Caractéristiques utiles :

- sortie analogique single-ended, simple à raccorder au seul ADC disponible ;
- alimentation 2,4 à 3,6 V ;
- sensibilité typique -38 dBV à 94 dB SPL ;
- SNR 68 dB(A) ;
- surcharge acoustique 130 dB SPL ;
- consommation typique environ 110 microampères ;
- composant actif et recommandé par le fabricant au moment de l'étude.

La variante automobile IM68A130A apporte une qualification et une tenue environnementale renforcées. Pour le prototype, la disponibilité sur carte flexible ou module d'évaluation peut être plus importante que la qualification.

Pourquoi ce choix :

- un seul signal analogique ;
- alimentation possible depuis le 3,3 V de la carte fille ;
- dynamique suffisante à proximité des hélices ;
- le bruit propre du microphone sera probablement bien inférieur au bruit aérodynamique réel.

### 8.2 Choix premium : IM73A135

[Infineon IM73A135](https://www.infineon.com/part/IM73A135)

Caractéristiques utiles :

- sortie analogique différentielle ;
- SNR 73 dB(A) ;
- surcharge acoustique 135 dB SPL ;
- IP57 au niveau du microphone ;
- appairage serré, intéressant pour une paire verticale ;
- alimentation maximale 3,0 V, donc rail propre 2,75 à 2,8 V nécessaire.

Ce microphone est préférable pour une version finale multi-microphones, mais il faut convertir sa sortie différentielle vers une entrée ADC single-ended ou utiliser un ADC différentiel externe. Il augmente donc le nombre de composants du MVP.

### 8.3 Alternative numérique PDM

Un microphone PDM, par exemple un modèle Infineon IM69Dxxx ou TDK T5838, ne transmet pas ses échantillons par I2C. L'I2C peut éventuellement servir à la configuration ; les données audio passent par PDM, I2S ou TDM.

Avantages :

- meilleure immunité aux perturbations sur les pistes ;
- pas d'amplificateur analogique externe ;
- bonne cohérence de phase pour un réseau de microphones.

Inconvénients ici :

- interface PDM/I2S non directement exposée de manière pratique sur la MicroCAN v5 ;
- décimation logicielle ou périphérique supplémentaire ;
- partage de broches avec les fonctions existantes ;
- complexité inutile pour reconnaître une bande étroite à 2,8 kHz.

Conclusion : analogique vers ADC pour le MVP ; PDM seulement dans une nouvelle révision matérielle si les essais le justifient.

### 8.4 Composants ST à éviter pour un nouveau design

Les microphones ST MP23ABS1 et IMP23ABSU sont intéressants techniquement, mais leurs pages produit les indiquaient `NRND` lors de l'étude. Ils ne sont donc pas recommandés pour lancer une nouvelle carte.

## 9. Conditionnement analogique du microphone

La sensibilité -38 dBV à 94 dB SPL correspond à environ 12,6 mV RMS.

Ordres de grandeur idéaux :

- 94 dB SPL : environ 12,6 mV RMS à la sortie du microphone ;
- 101 dB SPL : environ 28 mV RMS ;
- 110 dB SPL : environ 80 mV RMS ;
- 120 dB SPL : environ 252 mV RMS.

### 9.1 Connexion directe retenue, sans OPAMP

L'IM68A130(A) contient déjà son préamplificateur et fournit une **tension analogique single-ended polarisée**, pas la sortie brute de la membrane. Sa tension continue typique, proche de 1,3 V, est dans la plage 0–3,3 V de l'ADC ; il n'est donc pas nécessaire d'ajouter un condensateur de liaison et un pont de polarisation.

Connexion de départ pour chaque voie :

```text
3V3 analogique/filtré -- découplage local recommandé par Infineon -- VDD micro
GND continu ---------------------------------------------------- GND micro
OUT micro -- R série faible --+------------------------------- entrée ADC
                              |
                              C optionnel
                              |
                             GND
```

Principes :

- aucun OPAMP interne ou externe ;
- deux réseaux strictement identiques pour préserver la comparaison de phase et de niveau ;
- résistance série de quelques centaines d'ohms pour isoler/protéger la sortie ;
- empreinte de condensateur à la broche ADC pour former un RC passif et servir de réservoir de charge ;
- commencer avec des valeurs prudentes puis vérifier à l'oscilloscope le temps d'établissement et l'absence d'oscillation ; une plage expérimentale de 4,7 à 22 nF est raisonnable, la valeur finale dépendant de la résistance série et de l'impédance de sortie du microphone ;
- conserver un temps d'échantillonnage ADC assez long, provisoirement 47,5 cycles ;
- détecter numériquement les échantillons proches de 0 et du maximum.

Le RC passif limite une partie du bruit hors bande mais ne constitue pas un filtre anti-repliement très raide. Il ne faut pas choisir une coupure étroite autour de 2,8 kHz avant d'avoir mesuré la balise. La sélection 2,0–3,0 kHz reste numérique.

### 9.2 Conséquence de l'absence de gain

Sans gain analogique, le signal n'occupe qu'une petite partie des 12 bits de l'ADC. Cela donne en contrepartie une grande marge contre le bruit des hélices et les chocs acoustiques. Le suréchantillonnage matériel x4 ou x16, puis l'intégration étroite du Goertzel, permettent de réduire le bruit de quantification sans ajouter de RAM.

Le suréchantillonnage ne remplace toutefois pas un gain : il ne change ni la pleine échelle 0–3,3 V ni le niveau acoustique de surcharge. Si les essais montrent que le bruit analogique ou celui du microphone domine déjà la quantification, les bits supplémentaires seront peu utiles. Il faudra alors reconsidérer un AFE, mais seulement sur la base de mesures.

Implantation mécanique du microphone :

- port acoustique protégé par une mousse anti-vent ;
- membrane acoustique hydrophobe si possible ;
- ne jamais recouvrir le port par le vernis de tropicalisation ;
- éloigner l'entrée analogique du buck, du CAN et des pistes numériques rapides ;
- placer le microphone dans l'ombre acoustique du corps du module, sans enfermer le port dans une cavité résonante ;
- tester plusieurs orientations par rapport au flux d'air descendant.

## 10. Choix des capteurs lumineux

### 10.1 Choix MVP : TI OPT4048 sur I2C

[TI OPT4048](https://www.ti.com/product/OPT4048)

Caractéristiques utiles :

- quatre canaux : XYZ tristimulus et large bande ;
- bon rejet infrarouge ;
- temps de conversion configurable à partir de 600 microsecondes par canal ;
- dynamique annoncée jusqu'à environ 144 klux ;
- alimentation 1,6 à 3,6 V ;
- E/S tolérantes à 5,5 V ;
- jusqu'à quatre adresses sur un même bus I2C ;
- interruption matérielle disponible.

À une cadence de quelques flashs par seconde et à faible hauteur, il est suffisamment rapide pour un premier prototype. Les canaux couleur permettent de calculer une mesure sensible au rouge et moins sensible aux changements globaux de luminosité.

Utilisation recommandée :

- un capteur orienté vers le bas pour commencer ;
- éventuellement trois ou quatre capteurs orientés vers le bas et en oblique ;
- conversion des données brutes en unités cohérentes en tenant compte de l'exposant/gamme automatique ;
- conserver un drapeau de saturation et les changements de gamme ;
- utiliser la composante pulsée, pas seulement la valeur absolue de lux.

Avec plusieurs capteurs sur un module susceptible de tourner, utiliser de préférence :

- le maximum des amplitudes normalisées ; ou
- une somme calibrée des amplitudes ;

plutôt qu'un secteur directionnel qui deviendrait faux si l'orientation du module n'est pas connue.

### 10.2 Photodiode visible VEMD4200FX01

[Vishay VEMD4200FX01](https://www.vishay.com/en/product/84950/)

- photodiode visible en boîtier 0805 ;
- zone sensible 0,42 mm² ;
- bande à mi-sensibilité environ 400 à 660 nm ;
- rejet d'une grande partie du proche infrarouge solaire ;
- angle de demi-sensibilité environ +/-55 degrés ;
- réponse très rapide ;
- qualification automobile.

Elle est bien adaptée à une chaîne TIA qui doit éviter la saturation solaire. Sa faible surface réduit aussi le signal utile ; à valider avec les LED réelles.

### 10.3 Photodiode BPW34

[Vishay BPW34](https://www.vishay.com/en/product/81521/)

- grande surface sensible de 7,5 mm² ;
- très bonne sensibilité ;
- angle large ;
- réponse rapide ;
- sensibilité de 430 à 1100 nm.

Elle capte donc beaucoup d'infrarouge solaire. Elle nécessite un filtre rouge plus un coupe-IR et un TIA correctement dimensionné. Elle est intéressante si le signal utile est trop faible pour le VEMD4200.

### 10.4 TIA et traitement analogique optique

Amplificateurs possibles :

- [TI OPA381/OPA2381](https://www.ti.com/product/OPA381), spécialisé transimpédance, faible courant de polarisation et récupération rapide après surcharge ;
- [TI OPA320/OPA2320](https://www.ti.com/product/OPA320), rail-to-rail, faible courant de polarisation et adapté aux ADC basse tension.

Chaîne recommandée :

```text
photodiode
  -> TIA faible gain ne saturant pas au soleil
  -> mesure continue du fond et drapeau de saturation
  -> passe-haut lent, environ 0,5 à 1 Hz
  -> gain AC x16 à x32
  -> passe-bas 30 à 100 Hz
  -> ADC
```

Les valeurs du TIA doivent être déterminées après mesure du courant sous plein soleil et devant la balise. Une résistance de contre-réaction énorme choisie uniquement pour gagner de la sensibilité est une mauvaise idée : la sortie restera saturée et le flash sera invisible.

### 10.5 Capteurs I2C génériques à éviter

Les capteurs de lux génériques à temps d'intégration long, tels que certaines cartes BH1750 ou VEML7700, ne sont pas le premier choix :

- intégration pouvant moyenner le flash ;
- changements de gain automatiques difficiles à interpréter ;
- saturation ou récupération lente au soleil ;
- absence d'information couleur ou de forme temporelle suffisamment rapide.

L'OPT4048 a été retenu précisément parce qu'il est rapide, colorimétrique et possède une grande dynamique.

## 11. Acquisition audio continue sur le STM32G491

### 11.1 Ne jamais stocker toute la mission

Pour un microphone à 24 kéch/s et 16 bits :

\[
24000 \times 2 = 48000\ \text{octets/s}
\]

Pour deux microphones : 96 ko/s. Ce débit n'est pas un besoin de RAM : c'est seulement la quantité de données qui traverse successivement le même tampon.

Le principe est un tampon ping-pong ou circulaire :

```text
ADC -> DMA circulaire
       [ moitié A | moitié B ]
          traiter    remplir
          remplir    traiter
```

Le callback de demi-transfert signale qu'une moitié est prête. Le callback de transfert complet signale l'autre moitié. Le calcul doit être fait dans un thread ou une job queue ; l'interruption ne doit faire qu'envoyer l'index du bloc prêt.

Un simple booléen `block_ready` peut perdre silencieusement un demi-bloc si le traitement prend du retard. Utiliser un sémaphore, une mailbox, une file d'événements ou au minimum un compteur monotone avec compteur d'overrun.

### 11.2 Taille des buffers audio

À 24 kéch/s :

- tampon circulaire de 1024 échantillons `uint16_t` : 2048 octets ;
- chaque moitié : 512 échantillons, soit 21,33 ms ;
- le traitement dispose de 21,33 ms avant la réutilisation de cette moitié.

Avec deux microphones empaquetés dans un mot de 32 bits :

- 1024 couples : 4096 octets ;
- chaque moitié contient 512 échantillons par microphone ;
- même durée de 21,33 ms à 24 kéch/s.

Dans l'architecture non-dual retenue, on utilise en pratique deux tableaux indépendants de 1024 `uint16_t`, soit également 4096 octets au total. Les demi-buffers sont appariés par un numéro de génération avant traitement.

### 11.3 Budget RAM indicatif

| Élément | MVP 1 micro | Version 2 micros |
|---|---:|---:|
| DMA audio circulaire | 2 Kio | 4 Kio |
| Tampon(s) de calcul | 1 à 2 Kio | 2 à 4 Kio |
| Coefficients et états des filtres | < 1 Kio | < 2 Kio |
| Historique enveloppe/cadence | < 1 Kio | < 2 Kio |
| DMA optique/I2C | < 1 Kio | < 1 Kio |
| Total spécifique détection | environ 5 Kio | environ 10 Kio |

Un historique brut optionnel de 250 ms prendrait :

- un microphone : 12 Kio ;
- deux microphones : 24 Kio.

Cet historique ne doit servir qu'au débogage pré-trigger et peut être désactivé en compétition.

### 11.4 État mémoire réel de la branche

Le linker répartit les 112 Kio ainsi :

- SRAM1 : 80 Kio ;
- SRAM2 : 16 Kio ;
- CCM SRAM : 16 Kio ;
- SRAM1 + SRAM2 continues : 96 Kio ;
- CCM aliasée après SRAM2 à `0x20018000`.

La configuration actuelle réserve notamment :

- `CH_HEAP_SIZE = 20 Kio` ;
- `DMA_HEAP_SIZE = 12 Kio` ;
- `UAVNODE_MEMORYPOOL_SIZE = 6 Kio` ;
- piles principale et processus ;
- mémoire du nœud CAN, MFS, console et rôles existants.

Sur le binaire présent lors de cette étude :

```text
text   = 223184 octets
data   =   3024 octets
bss    = 100112 octets
```

Le `bss` inclut des zones réservées aux heaps et aux piles ; il ne signifie pas que les 100 Kio contiennent en permanence des données utiles. Il indique néanmoins qu'il faut contrôler le fichier `.map` et les heaps avant d'ajouter des dizaines de kilo-octets statiques.

Recommandation :

- tampon DMA audio statique dans `DMA_SECTION` pour obtenir une erreur de link immédiate en cas de dépassement ; ou allocation via `malloc_dma()` avec erreur propre au démarrage du rôle ;
- ne jamais placer le tampon sur la pile d'un thread ;
- alignement au minimum 16 bits, idéalement 8 ou 16 octets selon les macros existantes ;
- conserver les coefficients, états DSP et piles non-DMA dans la CCM si utile ;
- vérifier `mem`, le `.map`, l'utilisation des stacks et le DMA heap dans une configuration de rôles identique à celle de la mission.

Forme statique adaptée aux macros existantes :

```cpp
static IN_DMA_SECTION_NOINIT(adcsample_t audioSamples[1024]);
```

Traiter directement les demi-tampons autant que possible. Une copie complète en `float` n'est pas obligatoire : les biquads, Goertzel et accumulateurs peuvent fonctionner en Q15/Q31 ou convertir échantillon par échantillon.

Le projet place `DMA_SECTION` en SRAM normale. La CCM est accessible au DMA seulement par son adresse aliasée, comme décrit par [ST AN4296](https://www.st.com/resource/en/application_note/an4296-use-stm32f3stm32g4-ccm-sram-with-iar-embedded-workbench-keil-mdkarm-stmicroelectronics-stm32cubeide-and-other-gnubased-toolchains-stmicroelectronics.pdf). Pour éviter toute ambiguïté, les buffers DMA doivent rester dans la section déjà prévue par le projet.

### 11.5 Suréchantillonnage matériel ADC

Le STM32G491 peut accumuler plusieurs conversions dans l'ADC et ne produire qu'un seul résultat par événement de timer. **Le suréchantillonnage n'augmente donc ni le nombre d'échantillons DMA, ni la taille des buffers, ni le débit du DSP.** À 24 kéch/s, chaque voie reste un flux de 24 000 mots de 16 bits par seconde.

Sous ChibiOS 21.11 :

- le driver ADCv3 active automatiquement la prise en charge du suréchantillonnage sur STM32G4 ;
- `ADCConversionGroup` possède déjà le champ `.cfgr2` ;
- `STM32_ADC_COMPACT_SAMPLES` vaut `FALSE` dans ce projet, donc `adcsample_t` est un `uint16_t` capable de transporter un résultat 13 ou 14 bits ;
- chaque ADC étant utilisé en mode indépendant, son groupe écrit son propre `CFGR2` ;
- il n'est pas nécessaire d'activer le mode ADC dual.

Réglages utiles :

| Objectif idéal | Rapport matériel | Décalage à droite | Plage de sortie | Coût DMA/RAM |
|---|---:|---:|---:|---:|
| environ +1 bit | x4 | 1 bit | 0 à 8190, 13 bits | inchangé |
| environ +2 bits | x16 | 2 bits | 0 à 16380, 14 bits | inchangé |

La loi idéale est :

\[
\Delta ENOB = \frac{1}{2}\log_2(N)
\]

Elle suppose un bruit ou un dithering non corrélé d'une conversion à la suivante. Le microphone et l'ADC en fourniront probablement assez pour gagner près d'un bit ; le gain réel de deux bits devra être mesuré. Les erreurs statiques de l'ADC, le bruit du microphone et le bruit aérodynamique ne disparaissent pas magiquement.

Configuration illustrative des deux `ADCConversionGroup` audio :

```cpp
constexpr uint32_t ovsX4Keep13Bits =
  ADC_CFGR2_ROVSE |
  (1U << ADC_CFGR2_OVSR_Pos) |  // codage 1 : rapport x4
  (1U << ADC_CFGR2_OVSS_Pos);   // somme / 2

constexpr uint32_t ovsX16Keep14Bits =
  ADC_CFGR2_ROVSE |
  (3U << ADC_CFGR2_OVSR_Pos) |  // codage 3 : rapport x16
  (2U << ADC_CFGR2_OVSS_Pos);   // somme / 4
```

Ne pas positionner `ADC_CFGR2_TROVS`. Avec `TROVS = 0`, un front TIM6 lance immédiatement les x4 ou x16 sous-conversions et un résultat final est écrit. Avec `TROVS = 1`, il faudrait x4 ou x16 fronts pour obtenir un résultat et le débit utile tomberait respectivement à 6 ou 1,5 kéch/s si le timer restait à 24 kHz.

`ROVSM` est sans effet utile dans cette architecture puisqu'aucune conversion injectée n'interrompt l'audio ; il peut rester à zéro. Les deux groupes doivent avoir exactement la même valeur de `cfgr2`.

Avec l'horloge ADC12 actuelle à 42,5 MHz et un échantillonnage de 47,5 cycles, une conversion 12 bits prend environ :

\[
(47{,}5 + 12{,}5) / 42{,}5\ \mathrm{MHz} = 1{,}41\ \mu s
\]

Donc :

- x4 : environ 5,65 µs par résultat ;
- x16 : environ 22,6 µs par résultat ;
- période à 24 kéch/s : 41,67 µs.

Le mode x16 tient encore avec une marge confortable. Le temps d'échantillonnage de 640,5 cycles utilisé pour les mesures internes lentes ne doit surtout pas être repris pour l'audio. Une valeur de 24,5 cycles serait également possible après validation du réseau RC ; 47,5 cycles est un départ conservateur sans buffer analogique.

Recommandation de mise au point :

1. démarrer avec x4/décalage 1 pour obtenir un format 13 bits et une forte marge de timing ;
2. comparer sur les mêmes enregistrements le bruit de fond, le score Goertzel et le clipping avec x16/décalage 2 ;
3. retenir x16 par défaut si la mesure montre un gain réel et aucun effet secondaire ; il ne coûte pas de RAM supplémentaire.

Le code DSP ne doit plus supposer une pleine échelle de 4095 ni retirer une constante fixe de 2048. Il doit retirer une moyenne continue propre à chaque microphone. La pleine échelle vaut environ 8190 en x4/shift1 ou 16380 en x16/shift2. Les fonctions de conversion VIN/température/VREF restent en 12 bits dans leur autre groupe, dont `.cfgr2 = 0`.

Le burst de sous-conversions réalise aussi une petite moyenne temporelle. À 2,8 kHz, son atténuation est négligeable pour les valeurs ci-dessus, et son retard est commun aux deux ADC ; il ne crée donc pas de déphasage relatif si les configurations sont identiques. Il ne remplace pas pour autant un vrai filtre anti-repliement analogique.

Enfin, les deux ADC et TIM6 doivent respecter les contraintes de synchronisation indiquées dans l'[errata STM32G491 ES0523](https://www.st.com/resource/en/errata_sheet/es0523-stm32g491xx4a1xx-device-errata-stmicroelectronics.pdf), section 2.6.9. Avec l'horloge ADC synchrone actuelle `AHB/4`, le contournement ST consiste à :

- donner la même configuration d'horloge à ADC1 et ADC2 ;
- les déclencher avec le même timer ;
- donner à ce timer le même rapport de prescaler `/4`, ou un multiple entier.

Pour TIM6 cadencé à 170 MHz, configurer le GPT à 42,5 MHz force donc `PSC = 3`. Un intervalle de 1771 tops fournit environ 23 997,74 Hz, soit seulement -94 ppm par rapport à 24 kHz. Le DSP doit employer cette fréquence réelle dans ses coefficients ou bins Goertzel.

Les sections 2.6.7 et 2.6.8 du même errata imposent aussi de se méfier du premier résultat après une longue pause ou un arrêt logiciel. Lors du passage audio -> santé -> audio, le thread doit soit désactiver/réactiver l'ADC comme proposé par ST, soit réaliser et jeter une conversion factice. Par sécurité, le premier résultat audio de chaque voie après relance est ignoré et le DSP reçoit explicitement un marqueur de discontinuité.

## 12. Traitement numérique audio

Le STM32G491 possède un Cortex-M4F à 170 MHz, des instructions DSP, une FPU, CORDIC et FMAC. À 24 kéch/s sur une ou deux voies, la charge est faible comparée à la capacité CPU.

Le FMAC peut accélérer un FIR ou un IIR, mais n'est pas nécessaire pour le premier firmware. CMSIS-DSP ou des biquads écrits proprement suffisent et seront plus faciles à mettre au point.

CMSIS-DSP et FMAC ne sont pas actuellement intégrés à ce chemin de traitement dans le projet. Le premier prototype le plus sûr est donc un Goertzel direct et quelques biquads. Le build présent est compilé avec `-Og`; une fois fonctionnel et instrumenté, comparer avec `RELEASE=fast` en vérifiant que les délais, résultats numériques et callbacks restent corrects.

### 12.1 Pipeline recommandé

Pour chaque demi-tampon :

1. convertir l'échantillon ADC en valeur signée en retirant le point milieu ;
2. supprimer lentement l'offset restant ;
3. couvrir la bande 2,0 à 3,0 kHz, avec une petite garde de part et d'autre ;
4. calculer l'énergie dans une grille de fréquences Goertzel suffisamment
   serrée pour suivre un chirp ;
5. calculer l'énergie dans des bandes latérales de référence ;
6. former un rapport entre l'énergie cumulée de la bande et l'énergie globale,
   complété par le rapport aux bandes voisines ;
7. extraire l'enveloppe de la bande ;
8. détecter les fronts de salve avec hystérésis ;
9. mémoriser seulement les horodatages des dernières salves ;
10. estimer les cadences proches de 2 et 3 Hz sans les rendre obligatoires ;
11. produire un niveau, une fréquence dominante et une confiance.

L'implémentation utilise une grille de 50 Hz entre 1,95 et 3,05 kHz pour la
bande utile avec sa garde. Avec la borne basse par défaut à 2 kHz, les
références basses sont à 1,65 et 1,75 kHz et les références hautes à 3,25 et
3,40 kHz.

Ces valeurs sont provisoires. Elles doivent être adaptées au spectre mesuré.

### 12.2 Fenêtrage

À 24 kéch/s, 512 échantillons représentent 21,33 ms et une résolution FFT nominale de 46,875 Hz. C'est un bon compromis pour suivre des salves courtes.

Une fenêtre de Hann réduit les fuites spectrales, mais un Goertzel à fréquence arbitraire peut aussi être appliqué sur le bloc. L'enveloppe et la cadence doivent persister entre les blocs ; leurs états ne sont jamais réinitialisés à chaque callback DMA.

### 12.3 Rejet du bruit moteur

Ne pas utiliser uniquement un seuil d'énergie à 2,8 kHz : une harmonique moteur peut tomber dans cette bande.

Combiner :

- rapport énergie de bande / énergie globale ;
- rapport à des bandes voisines et élévation au-dessus du bruit adaptatif ;
- confirmation sur deux blocs consécutifs ;
- déplacement éventuel du maximum spectral à l'intérieur du chirp ;
- cadences proches de 2 ou 3 Hz comme informations complémentaires ;
- éventuel régime moteur reçu du contrôleur de vol ;
- cohérence avec le flash lumineux ;
- évolution spatiale lors du déplacement.

Un notch construit à partir du RPM peut supprimer aussi l'alarme si les fréquences coïncident. Il vaut mieux réduire la confiance d'une composante exactement liée au moteur que supprimer toute la bande.

### 12.4 Détection de clipping et adaptation du gain

Compter le nombre d'échantillons proches des rails ADC dans chaque bloc. Publier un drapeau `audio_clipped` et réduire la confiance d'amplitude lorsqu'il est actif.

Si le gain est commutable :

- passer au gain faible dès qu'un clipping apparaît ;
- ne remonter le gain qu'après une temporisation ;
- publier le gain utilisé afin que le contrôleur de vol normalise la carte.

## 13. Traitement optique

Pour chaque capteur :

1. lire les canaux couleur et large bande ;
2. convertir la donnée en tenant compte de la gamme et du temps d'intégration ;
3. maintenir une estimation lente du fond ;
4. soustraire ce fond pour obtenir la composante pulsée ;
5. calculer une mesure de rouge ou une chromaticité ;
6. détecter fronts, durée et amplitude des impulsions ;
7. rejeter les variations trop lentes dues à l'inclinaison, aux nuages ou à l'ombre du drone ;
8. rejeter autant que possible le 50/100/120 Hz et les ombres rapides d'hélices ;
9. comparer la cadence au gabarit réel de la balise ;
10. produire amplitude, cadence, saturation et confiance.

La cadence lumineuse ne doit pas être supposée égale à trois flashs par seconde tant qu'elle n'a pas été mesurée.

Les propres feux du drone doivent être :

- éteints pendant la recherche si possible ; ou
- horodatés par le firmware pour inhiber leurs impulsions ; ou
- physiquement masqués vis-à-vis du capteur suspendu.

## 14. Fusion son/lumière

Utiliser deux scores continus normalisés entre 0 et 1 :

- `audio_score` ;
- `optical_score`.

Puis calculer `fusion_score` avec les règles suivantes :

- son et lumière concordants pendant 1 à 2 s : confiance forte ;
- son seul très caractéristique : confirmation après davantage de salves ;
- lumière seule très caractéristique : confirmation avec seuil plus élevé ou durée plus longue ;
- saturation ou clipping : ne pas transformer automatiquement en absence de balise ;
- hystérésis pour éviter les basculements rapides ;
- conserver séparément les niveaux physiques pour la cartographie.

Éviter :

```text
detected = audio_detected && light_detected
```

Préférer un système de score tolérant à la perte d'une modalité.

## 15. Localisation par cartographie d'amplitude

### 15.1 Répartition des responsabilités

Architecture recommandée :

- la MicroCAN reconnaît localement la signature et publie des mesures horodatées ;
- le contrôleur de vol ou calculateur embarqué, qui connaît déjà position, altitude et trajectoire, construit la carte et décide de la trajectoire ;
- aucun audio brut n'est transmis pendant la mission.

Cette répartition évite de transmettre la pose complète du drone vers la MicroCAN et conserve le facteur d'autonomie 1 puisque tous les calculs restent embarqués.

Une autre option est que la MicroCAN reçoive la pose par CAN et construise elle-même la carte. Elle est possible, mais ajoute un couplage et consomme inutilement de la RAM.

### 15.2 Horodatage

Chaque salve individuelle doit être horodatée. Il ne faut pas affecter un score intégré sur 2 s à la seule position de fin de fenêtre.

Exemple : à 2 m/s, une fenêtre de 2 s couvre 4 m. Sans horodatage des salves, cette erreur suffit à choisir un mauvais point.

Il faut définir :

- base de temps locale monotone ;
- manière de la relier au temps du contrôleur de vol ;
- latence ADC + DSP + publication ;
- timestamp correspondant au centre de chaque salve ou bloc.

### 15.3 Grille de recherche

Profil initial à tester :

1. grille grossière espacée de 4 à 5 m ;
2. vitesse de 1 à 2 m/s avec régime moteur aussi constant que possible ;
3. sélection du maximum lissé ;
4. croix ou spirale de rayon 2 à 4 m ;
5. pas de raffinement 0,5 à 1 m ;
6. vitesse 0,3 à 0,5 m/s ou stationnaires de 1 à 2 s ;
7. confirmation finale et association au mannequin par caméra.

Un pas grossier de 5 m reste plausible à cette hauteur : le signal idéal au pire point d'une cellule diminue de plusieurs décibels mais reste élevé. Les valeurs réelles dépendront des essais.

### 15.4 Construction de la carte

Ne pas cartographier le SPL total ni les lux bruts. Cartographier :

- amplitude de la bande sonore reconnue ;
- rapport bande/voisinage ;
- confiance de cadence ;
- amplitude des impulsions rouges après retrait du fond ;
- score fusionné ;
- gain et saturation.

Possibilités :

- grille 1 m de 50 x 50 cellules : environ 2500 cellules ;
- moyenne pondérée et nombre d'observations par cellule ;
- lissage spatial léger ;
- conservation des meilleurs points seulement ;
- ajustement robuste d'un pic 2D autour du maximum.

Une carte 1 m avec 8 octets par cellule prend environ 20 Kio. Elle tient dans le G491, mais il est préférable de la stocker sur le calculateur de vol pour ne pas rogner les heaps du nœud CAN.

## 16. Messages DroneCAN proposés

Pour le prototype, un message de débogage ou tunnel peut suffire. Pour la version finale, créer un type DSDL dédié après avoir choisi le namespace et l'identifiant conformément aux conventions du projet.

Le build par défaut utilise le DSDL externe référencé par `microcan/Makefile` (`../UAVCAN/DSDL` relativement à l'arborescence de travail), et pas automatiquement la copie sous `ext/UAVCAN`. Cette dépendance doit être recréée ou corrigée lors de la reprise sur une autre machine.

Champs proposés :

```text
uint64 timestamp_usec
float16 audio_level
float16 audio_snr_like
float16 audio_frequency_hz
float16 audio_cadence_hz
float16 optical_level
float16 optical_red_ratio
float16 fusion_score
uint8 audio_gain
uint8 flags
```

Flags possibles :

- audio signature présente ;
- flash présent ;
- confirmation fusionnée ;
- clipping audio ;
- saturation optique ;
- gain automatique en transition ;
- module suspendu instable ;
- capteur en erreur.

Fréquence de publication : 20 à 50 Hz. Les événements de salve peuvent éventuellement être publiés séparément si le timestamp précis est nécessaire.

Ne pas transmettre le flux audio brut sur CAN pendant la mission. À deux microphones et 24 kéch/s, il représenterait environ 96 ko/s hors encapsulation et consommerait inutilement une grande partie du bus classique.

Le Makefile utilise par défaut `CAN_BITRATE=1000`, soit CAN classique à 1 Mbit/s dans cette configuration. Le brut stéréo représente déjà 768 kbit/s avant les en-têtes CAN/DroneCAN, les réémissions et le reste du trafic : cette transmission serait techniquement très fragile même avant de considérer la charge CPU.

## 17. Intégration logicielle sous forme de rôle

Nom provisoire : `ImavBeaconRole`.

Paramètre d'activation possible :

```text
ROLE.imav.beacon
```

Paramètres runtime possibles :

```text
role.imav.audio.sample_rate
role.imav.audio.gain
role.imav.audio.band_low_hz
role.imav.audio.band_high
role.imav.audio.threshold
role.imav.audio.cadence_min
role.imav.audio.cadence_max
role.imav.optical.threshold
role.imav.optical.integration_time
role.imav.fusion.threshold
role.imav.publish_frequency
```

`role.imav.audio.band_low_hz` est désormais implémenté : entier persistant
borné entre 2 000 et 2 600 Hz, avec 2 000 Hz par défaut. Il est lu au démarrage
du rôle ; un redémarrage de la MicroCAN est donc nécessaire après modification.
Une carte qui possède déjà la valeur 2 600 Hz en mémoire persistante la
conservera après mise à jour : il faudra lui écrire explicitement 2 000 Hz et
la redémarrer.
Les autres paramètres de cette liste restent des pistes tant qu'ils ne figurent
pas dans `nodeParameters.hpp`.

Le rôle devra :

- hériter de `RoleBase` et utiliser le schéma CRTP/trampoline existant ;
- acquérir PA3, PA4, ADC1, ADC2 et TIM6 ;
- acquérir I2C1 via l'infrastructure existante si l'OPT4048 est actif ;
- allouer deux tampons DMA 16 bits dans la section/heap DMA ;
- utiliser un thread unique pour posséder les ADC, commuter les groupes audio/santé et lancer le DSP ;
- armer les deux ADC avant de démarrer TIM6 et attendre les deux indicateurs de demi-buffer portant le même numéro avant de traiter une fenêtre stéréo ;
- ne faire presque aucun calcul dans le callback ADC ;
- publier les scores sans boucle d'attente bloquante ;
- signaler proprement conflit de ressources, manque de mémoire DMA et capteur absent ;
- être désactivable afin de conserver le firmware update et le diagnostic.

Le gestionnaire de ressources doit représenter explicitement :

```text
ADC_1
ADC_2
TIM_6
```

La surveillance VIN/température/VREF ne doit plus posséder ADC1 de son côté : elle devient une opération du même service/thread ADC ou lui adresse une requête.

Attention : I2C1 partage PB7 avec certaines fonctions TIM3/LED. Le rôle IMAV ne pourra pas coexister avec les rôles qui réaffectent PB7.

## 18. Suspension sous le drone

Le petit poids maintient le module approximativement vertical, mais ne supprime pas :

- le mouvement pendulaire ;
- la rotation autour du câble ;
- l'inclinaison sous accélération ;
- le déplacement horizontal entre le centre du drone et le capteur ;
- le flux d'air descendant.

### 18.1 Mesure de l'attitude du module

Ajouter si possible un petit IMU 6 axes sur I2C permet de :

- rejeter les mesures prises pendant une oscillation importante ;
- connaître l'inclinaison du module ;
- approximer la position du capteur avec la longueur du câble ;
- détecter un choc ou un accrochage ;
- interpréter plusieurs capteurs optiques orientés.

Une simple accélération ne donne une verticale fiable qu'en régime quasi statique. Un gyroscope améliore la détection des fenêtres stables.

### 18.2 Stabilisation mécanique

À comparer pendant les essais :

- suspension courte avec poids sous le module ;
- suspension à deux points pour limiter le lacet ;
- petit tube carbone vertical ;
- empennage passif ;
- amortissement mécanique ;
- vitesse et accélérations limitées pendant les mesures.

Une suspension longue éloigne le micro des moteurs, mais rapproche le système du sol et augmente le risque d'accrochage. La hauteur déclarée par le contrôleur de vol n'est pas la hauteur réelle du capteur.

### 18.3 Câblage

Les quatre fils externes restent :

- VBUS ;
- GND ;
- CAN-H ;
- CAN-L.

Recommandations :

- CAN-H/CAN-L torsadés ;
- anti-traction aux deux extrémités ;
- câble souple mais pas au point de fouetter les capteurs ;
- masse et alimentation correctement dimensionnées ;
- connecteur verrouillable ;
- vérifier la terminaison 120 ohms selon la position réelle du nœud sur le bus ;
- garantir la garde au sol du capteur et du poids ;
- empêcher toute interaction avec le mécanisme de largage.

## 19. Journalisation et débogage

En mission, ne publier que les caractéristiques compactes.

Pendant la mise au point, prévoir un mode de capture :

- tampon circulaire brut de 100 à 250 ms ;
- gel du tampon lors d'une détection ou d'un faux positif ;
- récupération après vol par UART, USB de sonde ou CAN au sol ;
- stockage facultatif dans la M95P seulement après l'arrêt de l'acquisition et si l'endurance mémoire est acceptable ;
- métadonnées : gain, régime moteur, attitude, position, saturation et conditions de test.

Le flux brut continu sur CAN n'est pas nécessaire. Sur banc, il peut être utile ponctuellement si le débit et la configuration du bus le permettent.

## 20. Plan de prototypage

### Étape 0 — caractériser la balise réelle

Mesurer :

- spectre et fréquence dominante ;
- variation de fréquence dans une salve ;
- durée et enveloppe des salves ;
- cadence préalarme et alarme ;
- SPL à 1, 1,5, 3, 5 et 10 m ;
- couleur et spectre approximatif des LED ;
- durée et cadence des flashs ;
- synchronisation son/lumière ;
- directivité sonore et lumineuse ;
- comportement selon orientation et occultation ;
- référence/date de fabrication vis-à-vis de l'avis MSA.

### Étape 1 — banc minimal

- MicroCAN v5 ;
- deux IM68A130(A) sur carte fille, sans OPAMP, avec réseaux RC passifs identiques ;
- sorties analogiques vers PA3/ADC1_IN4 et PA4/ADC2_IN17 ;
- PA2 conservée en TX de trace ;
- comparaison ADC sans suréchantillonnage, x4/shift1 et x16/shift2 ;
- OPT4048 sur I2C1 ;
- capture de blocs audio ;
- Goertzel et détection de cadence ;
- détection du flash et retrait du fond ;
- publication de scores de débogage.

### Étape 2 — moteurs arrêtés puis en fonctionnement

Tester :

- moteur arrêté ;
- moteurs armés au sol ;
- plusieurs régimes fixes ;
- stationnaire ;
- translations et accélérations ;
- différentes hélices ;
- mousse anti-vent et plusieurs orientations du microphone ;
- propres feux du drone allumés/éteints.

### Étape 3 — lumière extérieure

Tester :

- soleil face au capteur ;
- capteur dirigé vers le sol ;
- ciel couvert ;
- herbe claire/sombre ;
- surfaces réfléchissantes ;
- ombre mobile du drone ;
- orientation complète de la balise ;
- occultation par le mannequin ;
- plusieurs réglages de temps d'intégration OPT4048.

### Étape 4 — cartographie aveugle

- position de la balise tirée au hasard ;
- mannequin équipé inconnu du logiciel ;
- grille autonome ;
- raffinement autour du maximum ;
- mesure de l'erreur entre maximum estimé et balise ;
- répétitions avec différentes orientations et conditions.

Objectifs provisoires :

- détection > 95 % dans l'enveloppe de vol choisie ;
- moins d'un faux candidat par heure ;
- confirmation bimodale en moins de 3 s ;
- confirmation monomodale en moins de 5 s ;
- erreur de localisation compatible avec l'identification du bon mannequin ;
- aucune saturation silencieuse : tout clipping doit être signalé.

Ces objectifs doivent être validés sur au moins 20 à 30 répétitions des cas critiques.

## 21. Principaux risques et parades

| Risque | Conséquence | Parade |
|---|---|---|
| Harmonique moteur vers 2,8 kHz | Faux positif ou masquage | Rapport bande/global, bandes voisines, niveau absolu et confirmation optique |
| Vent sur le microphone | Saturation basse fréquence | Mousse, passe-haut, placement |
| Bruit hélices malgré l'absence de gain | Clipping ou masque de la balise | Marge directe micro->ADC, drapeau clipping, comparaison verticale |
| Soleil direct | Saturation optique | Capteur vers le sol, couleur, retrait du fond, TIA faible gain |
| LED occultées | Absence de lumière | Accepter le son seul, passages sous plusieurs angles |
| Alarme sonore défaillante | Absence de son | Accepter la lumière seule, avis MSA |
| Module suspendu en oscillation | Erreur de position/niveau | IMU, vitesse faible, rejeter fenêtres instables |
| Rotation du module | Secteurs optiques faux | Somme/max, IMU, suspension anti-lacet |
| Fenêtre DSP trop longue en mouvement | Carte décalée | Horodater chaque salve, ralentir au raffinement |
| RAM saturée par les rôles/heaps | Échec au démarrage | Deux tampons totalisant 4 Kio, contrôle `.map`, allocation DMA vérifiée |
| PA3 réutilisée en ADC | Perte de `DBG_RX` | Conserver PA2 comme trace TX minimale |
| I2C1 réaffecté par un autre rôle | Perte OPT4048 | Gestion de ressources et configuration mission dédiée |
| Firmware update pendant acquisition SPI future | Conflit de bus | Arrêter rôle avant update, arbitrage explicite |
| Source lumineuse rouge parasite | Faux positif | Cadence, mouvement spatial, fusion audio |
| Autre alarme sonore | Faux positif | Gabarit spectral/temporel et confirmation spatiale |

## 22. Décision recommandée aujourd'hui

Construire d'abord une petite carte fille :

1. **deux IM68A130/IM68A130A analogiques**, alimentés et découplés proprement ;
2. aucun OPAMP ; deux sorties déjà polarisées directement vers **PA3/ADC1_IN4** et **PA4/ADC2_IN17** au travers de réseaux RC identiques ;
3. PA2 conservée comme `DBG_TX` minimal ;
4. **OPT4048** sur le connecteur I2C1 ;
5. empreinte optionnelle pour un IMU I2C ;
6. mécanique permettant de changer l'écart vertical, la mousse, l'orientation et la longueur de suspension ;
7. points de test sur les deux sorties analogiques ;
8. aucune photodiode/TIA complexe sur la première révision, sauf empreinte expérimentale si la place le permet.

Implémenter ensuite un rôle MicroCAN minimal :

- ADC1 et ADC2 indépendants, déclenchés ensemble par TIM6 à environ 24 kéch/s ;
- deux DMA circulaires de 1024 x 16 bits, soit 4 Kio au total ;
- blocs appariés de 512 échantillons par microphone ;
- suréchantillonnage matériel x4/shift1 au bring-up puis comparaison x16/shift2 ;
- un seul thread d'acquisition, avec pause explicite pour VIN/température/VREF et rejet du premier résultat après reprise ;
- passe-bande 2,4–3,2 kHz ;
- Goertzel multi-fréquences ;
- enveloppe et cadence ;
- lecture OPT4048 ;
- score fusionné et publication 20–50 Hz ;
- capture pré-trigger facultative de 100–250 ms.

Ne passer à un AFE actif, un ADC externe ou à des photodiodes analogiques que si les essais montrent que cette chaîne directe est insuffisante. Cette séquence réduit fortement le risque de construire une carte complexe autour d'hypothèses que le règlement ne spécifie pas.

## 23. Questions ouvertes pour la prochaine session

- Peut-on obtenir un motionSCOUT K-T-R réel avant de dessiner la carte fille ?
- Quelle est la hauteur réelle du capteur suspendu, et non celle du drone ?
- Quelle longueur de suspension et quelle garde au sol sont acceptables ?
- Le drone dispose-t-il déjà d'une caméra capable d'identifier les trois mannequins ?
- Quels rôles MicroCAN seront actifs simultanément pendant cette mission ?
- TIM6 est-il libre dans la configuration exacte de mission et son TRGO est-il validé sur ADC1 et ADC2 par un test de phase ?
- Le driver ADC ChibiOS actuel fournit-il exactement les callbacks half/full souhaités sur les deux drivers indépendants ?
- Quel message DSDL/identifiant utiliser pour publier les scores ?
- Comment synchroniser précisément le timestamp MicroCAN avec la pose du contrôleur de vol ?
- L'OPT4048 peut-il voir les flashs sous plein soleil avec l'orientation réelle ?
- La paire directe sans gain conserve-t-elle assez de SNR en stationnaire ?
- Le suréchantillonnage x16 apporte-t-il réellement plus que x4 dans le bruit du drone ?
- Quelle valeur RC donne le meilleur compromis établissement/anti-repliement sans charger la sortie des microphones ?
- La mémoire DMA heap disponible dans la configuration mission permet-elle l'allocation du tampon sans réduire d'autres rôles ?
- Faut-il désactiver les propres feux du drone pendant la recherche ?

## 24. Sources techniques

### Sources locales

- [Règlement IMAV 2026 v2.1](Rulebook_IMAV2026_V2.1.pdf)
- [Schéma MicroCAN v5](../HARDWARE/MICROCAN/MicroCan_v5_schematic.pdf)
- [Brochage MicroCAN](../microcan/cfg/MICROCAN.cfg)
- [Configuration MCU ChibiOS](../microcan/cfg/mcuconf.h)
- [Linker STM32G491](../microcan/cfg/STM32G491xE.ld)
- [Surveillance ADC existante](../COMMON/source/adcSurvey.cpp)
- [Gestionnaire de ressources](../COMMON/source/resourceManager.hpp)
- [Architecture des rôles](../docs/software/adding_roles.md)

### Sources fabricants

- [STM32G491 — page produit ST](https://www.st.com/en/microcontrollers-microprocessors/stm32g491mc.html)
- [STM32G491 — datasheet DS13122](https://www.st.com/resource/en/datasheet/stm32g491rc.pdf)
- [STM32G4 — manuel de référence RM0440](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32G491 — errata ES0523](https://www.st.com/resource/en/errata_sheet/es0523-stm32g491xx4a1xx-device-errata-stmicroelectronics.pdf)
- [ST AN5537 — suréchantillonnage ADC](https://www.st.com/resource/en/application_note/an5537-how-to-use-adc-oversampling-techniques-to-improve-signaltonoise-ratio-on-stm32-mcus-stmicroelectronics.pdf)
- [ST AN4296 — utilisation de la CCM SRAM](https://www.st.com/resource/en/application_note/an4296-use-stm32f3stm32g4-ccm-sram-with-iar-embedded-workbench-keil-mdkarm-stmicroelectronics-stm32cubeide-and-other-gnubased-toolchains-stmicroelectronics.pdf)
- [MSA motionSCOUT — fiche produit](https://s7d9.scene7.com/is/content/minesafetyappliances/motionSCOUT%20Bulletin%20-%20ES)
- [MSA motionSCOUT — avis de service février 2026](https://assetlibrary.msasafety.com/m/1daafc2b15a7c2a5/original/Avis-de-service-Dispositif-MSA-motionSCOUT-PASS-Fevrier-2026.pdf)
- [Infineon IM68A130](https://www.infineon.com/part/IM68A130)
- [Infineon IM73A135](https://www.infineon.com/part/IM73A135)
- [TI OPT4048](https://www.ti.com/product/OPT4048)
- [Vishay VEMD4200FX01](https://www.vishay.com/en/product/84950/)
- [Vishay BPW34](https://www.vishay.com/en/product/81521/)
- [TI OPA381](https://www.ti.com/product/OPA381)
- [TI OPA320](https://www.ti.com/product/OPA320)

## 25. État de l'implémentation au 22 juillet 2026

La branche active est `imav2026`. Un premier rôle fonctionnel `ImavRole` est maintenant intégré, compilé par `USE_IMAV_ROLE` et activé à l'exécution par `ROLE.imav.beacon`. Aucun autre rôle n'a été désactivé statiquement.

### 25.1 Acquisition et partage des ressources

- PA3/ADC1_IN4 est acquis avec ADC1 et TIM6 par le gestionnaire de ressources ; PA4 et ADC2 ne sont pas utilisés par IMAV.
- ADC1 utilise une conversion régulière circulaire d'un canal, un DMA de 1024 mots et un callback minimal.
- TIM6 fournit le TRGO à `42,5 MHz / 1771 = 23 997,74 Hz` ; son prescaler `/4` respecte le contournement de l'errata avec l'horloge ADC AHB/4.
- Le suréchantillonnage est x4 avec shift 1 : sortie 13 bits, débit DMA inchangé.
- Un watchdog relance l'acquisition si aucun demi-buffer ne progresse pendant 100 ms.
- Toutes les secondes, le thread arrête le trigger puis ADC1, réalise la séquence lente de santé et relance l'audio. La séquence lente commence par un VREF dummy qui absorbe le premier résultat invalide décrit par ES0523 ; VIN, température et VREF utiles suivent dans la même séquence.
- Après chaque reprise, le premier demi-buffer audio est jeté et le suivant porte un marqueur de discontinuité.
- Si le démarrage du nœud ou d'un rôle échoue, ADC1 revient au mode historique continu afin que la santé carte ne reste pas figée.

PA2 reste utilisable pour `DebugTrace`. En mode IMAV, le shell n'est pas créé et le récepteur LPUART est réellement désactivé (`RE`, RXNE/error/LIN interrupts et FIFO RX), avant de passer PA3 en analogique.

### 25.2 DSP audio actuel

Chaque bloc mono de 512 échantillons est consommé immédiatement, sans historique brut :

1. moyenne, écart absolu moyen, minimum, maximum et clipping ;
2. fenêtre de Hann générée par récurrence, donc sans table RAM ;
3. grille Goertzel fixe de 50 Hz permettant de choisir une borne basse entre
   2,0 et 2,6 kHz ; la valeur par défaut 2,0 kHz utilise les références à
   1,65/1,75 et 3,25/3,40 kHz ainsi qu'une marge de 50 Hz autour de la bande ;
4. puissance moyenne de bande, proéminence sur les références, rapport en dB
   entre l'énergie cumulée de bande et l'énergie globale, et fréquence
   dominante indicative ;
5. plancher lent adaptatif et score continu par voie ;
6. machine `Unarmed/Off/On` à deux blocs avec hystérésis ;
7. six horodatages de fronts au maximum et mesure des cadences proches de 2 et
   3 Hz, sans utiliser cette cadence comme condition de validité.

L'état `Unarmed` est réimposé après une discontinuité. Il faut ensuite observer
deux blocs bas, puis deux blocs spectraux hauts consécutifs pour valider le
signal. Le score est maintenu pendant 750 ms puis décroît jusqu'à 1 500 ms afin
de couvrir les silences de la préalarme et de l'alarme. Une tonalité continue
fortement concentrée dans la bande peut donc être acceptée : c'est un choix
délibéré compte tenu du faible risque de leurre dans la mission.

Les seuils numériques actuels sont volontairement des valeurs de bring-up. Ils doivent être recalés sur des enregistrements de la vraie balise et du drone, en conservant les scores continus plutôt qu'un simple booléen.

### 25.3 OPT4060, VL53L4CX et exclusion temporelle

- Un unique OPT4060 orienté vers le sol est recherché aux adresses 0x44 à 0x47 ; `ADDR=GND` et l'adresse 0x44 sont les valeurs nominales.
- Le registre de configuration vaut `0x30B8` en acquisition : plage automatique, 1,8 ms par voie, mode continu et quatre voies séquentielles. Un cycle RGBW typique dure donc 7,2 ms.
- Le thread lit toutes les 10 ms les huit registres de résultat par un unique transfert burst. Exposant et mantisse sont linéarisés en codes ADC 26 bits ; les compteurs de conversion empêchent de mélanger un cycle incomplet et permettent de détecter un capteur bloqué.
- Les coefficients TI `R=2,4×CH0`, `G=CH1` et `B=1,3×CH2` sont appliqués avant de calculer la dominance rouge de la variation. Le score instantané combine amplitude absolue, variation relative, élévation au-dessus du bruit et chromaticité rouge.
- Deux mesures hautes puis deux mesures basses valident les fronts. Six fronts
  au maximum alimentent un score acceptant 2 Hz pour la préalarme et 3 Hz pour
  l'alarme complète ; un flash isolé ou la LED d'état rouge à 1 Hz ne suffit
  pas à produire le score final `lit`.
- Le VL53L4CX utilise sans modification le composant officiel ST du sous-module `third_party/x-cube-tof1`, épinglé sur X-CUBE-TOF1 v3.4.3 (cœur VL53LX 1.2.13). Seuls les callbacks I2C/temps sont adaptés à ChibiOS.
- Il fonctionne en profil longue distance, budget de 30 ms et one-shot asynchrone borné à 100 ms. Le résultat valide le plus proche est publié.
- Avant chaque one-shot ou réinitialisation ToF, le mode de l'OPT4060 passe à `Power-down` par une écriture I²C acquittée. La mesure ST est effectuée, puis le mode continu est rétabli ; aucun résultat optique de cette fenêtre n'est utilisé.
- La période nominale vaut 200 ms et est paramétrable. Un petit jitter déterministe déplace la phase des trous afin de ne pas masquer systématiquement une balise périodique.
- I2C standard 100 kHz et Fast mode 400 kHz sont acceptés ; 400 kHz reste recommandé. Le Fast-mode Plus 1 MHz est refusé car l'OPT4060 passe directement du Fast mode au protocole High-Speed 2,6 MHz, non activé ici.
- Un NACK d'un capteur absent ne réinitialise pas le bus partagé. Seuls un timeout ou une faute électrique/protocolaire déclenchent la récupération sérialisée.

La distance est publiée à chaque mesure dans `uavcan.equipment.range_sensor.Measurement` (`sensor_id=0`, type LIDAR, champ de vue 18°). L'orientation de corps est laissée indéfinie car le module est suspendu ; le contrôleur de vol configure lui-même l'orientation du télémètre vers le bas.

La lumière et le son restent deux preuves indépendantes. Il n'y a pas de condition rigide `son ET lumière`, notamment à cause de l'avis MSA 2026 sur des alarmes sonores potentiellement défaillantes.

### 25.4 RAM, threads et télémétrie

Mesures contrôlées sur le build `-Og` :

- `sizeof(ImavAudioState) = 2 624` octets sur le heap DMA de 12 288 octets ; ce total contient le buffer audio mono de 2 048 octets ;
- `sizeof(ImavLightRange) = 10 304` octets sur le heap standard de 20 480 octets ; ce total contient le contexte officiel ST de 9 480 octets et les tampons I2C ;
- thread audio : pile utile configurée à 1 536 octets ;
- thread capteurs : pile utile configurée à 2 048 octets ;
- firmware : 276 472 octets de texte et 100 056 octets de BSS, heaps réservés inclus ;
- le shell de 2 000 octets n'est pas alloué en mode IMAV.

Le portage matériel local du VL53L4CX réutilise le tampon `tofTx` contenu dans
`ImavLightRange`. Le `_I2CBuffer[256]` du portage générique ST n'est donc plus
lié au firmware, sans modifier le sous-module officiel. La section `.bss`
applicative passe de 35 624 à 35 368 octets et les 256 octets libérés sont
rendus au heap par le linker. Le total BSS synthétique reste à 100 056 octets
précisément parce qu'il inclut ce heap redimensionné. Lorsque le rôle est
compilé mais désactivé, il ne reste en RAM que les deux pointeurs de trampoline
de 4 octets ; tous ses buffers et états sont alloués à son lancement.

La télémétrie temporaire `uavcan.protocol.debug.KeyValue`, activable par `role.imav.debug.publish`, diffuse environ une fois par seconde :

| Clé | Valeur |
|---|---|
| `a0` | score spectral instantané du microphone |
| `p0` | amplitude RMS estimée de la tonalité dominante, en comptes ADC 13 bits |
| `sdb` | rapport en dB entre l'énergie de la bande balise et l'énergie globale de la fenêtre |
| `aud` | score de signature spectrale audio, maintenu pendant les silences |
| `frq` | fréquence dominante en hertz |
| `cad` | cadence de salves estimée en hertz |
| `lit` | score de flash rouge qualifié par une cadence de 2 ou 3 Hz, zéro après 200 ms sans nouvel échantillon |
| `rng` | distance en mètres, -1 si la dernière mesure n'est pas valide |
| `rsg` | signal VL53L4CX en kcps/SPAD |

Les clés font trois caractères afin que chaque message tienne dans une seule trame CAN classique.

### 25.5 Validation encore nécessaire

- mesurer le temps CPU maximum du DSP sur cible en `-Og` puis en `RELEASE=fast` ;
- injecter des sinus/salves connus sur PA3 et vérifier fréquence, cadence, clipping et reprise après pause santé ;
- enregistrer la vraie forme temporelle du motionSCOUT et ajuster les seuils/profils de cadence ;
- mesurer la largeur réelle des flashs et vérifier qu'un allumage produit plusieurs cycles RGBW complets ;
- tester l'OPT4060 au soleil, à l'ombre, devant des LED parasites et avec l'ouverture mécanique définitive ;
- observer les écritures `Power-down` et l'émission 940 nm pour confirmer sur cible qu'il n'existe aucun recouvrement lumière/ToF ;
- qualifier le VL53L4CX vers 1 m au-dessus de sols clairs et sombres, en extérieur, avec le verre de protection et le balancement du fil ;
- configurer l'orientation vers le bas dans le contrôleur de vol et valider la consommation de `uavcan.equipment.range_sensor.Measurement` ;
- décider la fusion et les durées de confirmation monomodales/bimodales après ces essais ;
- surveiller les heaps dans la configuration exacte de mission avant d'envisager une désactivation statique d'autres rôles.
