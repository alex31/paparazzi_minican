# Détection IMAV 2026 du mannequin équipé d'un MSA motionSCOUT

> Note de conception et de reprise de contexte — 26 juillet 2026
> Projet : MicroCAN v5, branche `imav2026/one_mic_adc2`
> MCU : STM32G491KEU6, ChibiOS 21.11, DroneCAN/UAVCAN v0  
> Statut : première implémentation fonctionnelle ; les seuils restent à calibrer
> sur la balise réelle et sous le drone.

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

La deuxième conclusion est liée au matériel existant et aux décisions prises après l'étude initiale : **le MVP utilise un seul microphone analogique IM68A130(A), directement relié à PA4/ADC2_IN17, et un OPT4060 orienté vers le sol pour les flashs**. ADC2 est ainsi dédié au son, tandis qu'ADC1 conserve exclusivement la surveillance VIN/température/VREF. Le VL53L4CX de mesure de hauteur n'est pas monté et reste une option logicielle désactivée par défaut.

Les décisions suivantes font foi lorsqu'une section historique du document semble encore présenter une variante antérieure :

- un microphone unique sur PA4 = ADC2_IN17 ;
- PA2/PA3 conservées pour la console de débogage complète ;
- ADC2 dédié à l'audio et ADC1 dédié à la surveillance carte ;
- pas d'OPAMP interne ni externe dans le premier prototype ;
- liaison continue en tension entre la sortie déjà polarisée de l'IM68A130(A) et l'ADC, avec seulement découplage et petit réseau RC passif ;
- suréchantillonnage matériel ADC envisagé en x4 ou x16, sans augmentation de la taille des tampons DMA ;
- un seul OPT4060 sous le drone, sans sectorisation optique ;
- score lumineux combinant une voie rapide spectrale à mémoire finie et une
  voie DFT lente pour le signal lointain noyé dans le bruit ;
- VL53L4CX optionnel, piloté lorsqu'il est activé par le composant officiel ST provenant du sous-module `STMicroelectronics/x-cube-tof1` ;
- pas de canaux injectés ni d'arbitrage entre audio et mesures lentes ;
- acquisition lumière et mesure ToF strictement exclusives lorsque le ToF est activé.

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

En conservant I2C1, SPI1/M95P interne et CAN, une seule entrée analogique externe est retenue :

| Broche MCU | Connecteur/fonction actuelle | Fonction possible | Décision actuelle |
|---|---|---|---|
| PA2 | sonde J3, `DBG_TX` | `ADC1_IN3` possible | conservée pour une trace TX minimale |
| PA3 | sonde J3, `DBG_RX` | `ADC1_IN4` possible | conservée pour la console de débogage |
| PA4 | J6 broche 4, `SPI_PERIPH_CS` | `ADC2_IN17`, microphone unique | retenue ; perte du chip-select SPI externe et de TIM3_CH2 |

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

Le microphone possède déjà une sortie amplifiée et polarisée. Son réseau vers
PA4 reste donc passif : petite résistance série, découplage local et empreinte
de condensateur optionnelle. Aucun second canal audio n'est câblé.

### 6.2 ADC et timers dans le firmware actuel

Le firmware actuel :

- réserve ADC1 aux mesures VIN/température/VREF en conversion continue ;
- réserve ADC2 au microphone ;
- utilise ADC2 en conversion circulaire mono déclenchée par TIM6 ;
- utilise TIM2 pour l'OS ;
- utilise potentiellement TIM7 pour DShot ;
- utilise FDCAN2 pour le réseau ;
- utilise I2C1 sur PA15/PB7 pour l'OPT4060 et le VL53L4CX ;
- dispose d'un gestionnaire de ressources pour les rôles.

`main.cpp` démarre toujours la surveillance continue sur ADC1. Le rôle IMAV
démarre indépendamment ADC2 et ne suspend donc jamais les mesures lentes.

Séquence de fonctionnement :

1. configurer ADC2 avec un groupe régulier d'un canal sur PA4 ;
2. armer son DMA circulaire de 1024 mots, puis démarrer le TRGO TIM6 à
   23 997,74 Hz ;
3. traiter chaque demi-buffer de 512 échantillons dans le thread audio ;
4. laisser ADC1 mesurer en parallèle VREF factice, VIN, température et VREF ;
5. en cas d'erreur ou de blocage audio, arrêter puis relancer uniquement
   TIM6 et ADC2, jeter le premier demi-buffer et signaler la discontinuité au
   détecteur.

Cette architecture supprime les pauses audio périodiques et toute commutation
de groupe ADC. Les deux acquisitions restent indépendantes, hors horloge
commune ADC12 imposée par le STM32G491.

Le rôle acquiert PA4, ADC2 et TIM6 avant de démarrer. ADC1 n'appartient pas au
rôle IMAV et reste géré par la surveillance carte.
L'infrastructure I2C commune arbitre PA15/PB7 ; un conflit avec une fonction
TIM3 utilisant PB7 est donc signalé au démarrage.

Le fichier CubeMX `.ioc` est partiellement obsolète : il indique notamment 160 MHz, tandis que le firmware vise 170 MHz. Les fichiers ChibiOS, le schéma et le code source sont les références à privilégier.

## 7. Architecture recommandée par étapes

### 7.1 MVP compatible avec la MicroCAN existante

```text
IM68A130(A) -> RC passif -> PA4 / ADC2_IN17 -> DMA mono -+
                                                           |
OPT4060 RGBW -------------------------------> I2C1 --------+--> STM32G491
VL53L4CX ToF -------------------------------> I2C1 --------+    -> mesures
```

Cette architecture apporte déjà :

- reconnaissance de l'énergie sonore concentrée entre 2,0 et 3,0 kHz ;
- mesure de la force du signal sonore ;
- détection indépendante des flashs rouges ;
- mesure de la hauteur du module suspendu ;
- exclusion temporelle explicite entre lumière et ToF ;
- cartographie sur le contrôleur de vol ;
- conservation de PA2/PA3 pour la console et utilisation de PA4 sur J6 ;
- aucune perte de l'accès à la mémoire M95P.

Le son et la lumière restent deux preuves indépendantes. Leur accord augmente
la confiance, mais aucun `ET` rigide n'est imposé, notamment parce que certaines
motionSCOUT concernées par l'avis MSA peuvent perdre leur alarme sonore.

### 7.2 Capteurs uniques orientés vers le bas

Le module est suspendu sans orientation angulaire maîtrisée. Plusieurs secteurs
optiques ou une paire de microphones directionnelle compliqueraient donc le
prototype sans fournir immédiatement un azimut exploitable. Le signal utile
vient du sol ; le choix retenu est :

- un microphone unique protégé du vent ;
- un OPT4060 unique orienté vers le sol ;
- un VL53L4CX unique orienté vers le sol.

La localisation ne dépend pas d'un secteur instantané : le contrôleur de vol
associe les scores successifs à la position du drone et recherche leur maximum
spatial.

### 7.3 Extensions seulement si les essais l'exigent

Une photodiode avec TIA, un ADC externe, un second microphone ou une IMU ne
seront ajoutés que si les essais sous le drone montrent une insuffisance mesurée
du montage minimal. Aucune de ces extensions ne fait partie de la carte
actuelle.

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

Connexion retenue pour l'unique voie :

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
- un unique réseau passif entre le microphone et PA4 / ADC2_IN17 ;
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

### 10.1 Choix MVP : TI OPT4060 sur I2C

[TI OPT4060](https://www.ti.com/product/OPT4060)

Caractéristiques utiles :

- quatre canaux RGBW avec forte réjection du proche infrarouge ;
- alimentation et niveaux logiques compatibles avec le rail 3,3 V ;
- temps de conversion configurable, fixé à 1,8 ms par voie dans le firmware ;
- plage automatique pour supporter de fortes variations d'éclairement ;
- jusqu'à quatre adresses sur un même bus I2C ;
- interruption matérielle disponible.

Les quatre voies sont converties séquentiellement en environ 7,2 ms. Une
impulsion data-ready sur PA8 réveille le thread, qui les lit alors par burst à
une cadence pouvant approcher 139 Hz. Les canaux couleur permettent de
calculer une dominance rouge moins sensible aux changements globaux de lumière.

Utilisation recommandée :

- un seul capteur orienté vers le bas ;
- conversion des données brutes en unités cohérentes en tenant compte de l'exposant/gamme automatique ;
- conserver un drapeau de saturation et les changements de gamme ;
- utiliser la composante pulsée et la chromaticité, pas seulement la valeur
  absolue de lumière ;
- arrêter explicitement l'acquisition pendant chaque mesure VL53L4CX.

### 10.2 Alternative non retenue : photodiode visible VEMD4200FX01

[Vishay VEMD4200FX01](https://www.vishay.com/en/product/84950/)

- photodiode visible en boîtier 0805 ;
- zone sensible 0,42 mm² ;
- bande à mi-sensibilité environ 400 à 660 nm ;
- rejet d'une grande partie du proche infrarouge solaire ;
- angle de demi-sensibilité environ +/-55 degrés ;
- réponse très rapide ;
- qualification automobile.

Elle est bien adaptée à une chaîne TIA qui doit éviter la saturation solaire. Sa faible surface réduit aussi le signal utile ; à valider avec les LED réelles.

### 10.3 Alternative non retenue : photodiode BPW34

[Vishay BPW34](https://www.vishay.com/en/product/81521/)

- grande surface sensible de 7,5 mm² ;
- très bonne sensibilité ;
- angle large ;
- réponse rapide ;
- sensibilité de 430 à 1100 nm.

Elle capte donc beaucoup d'infrarouge solaire. Elle nécessite un filtre rouge plus un coupe-IR et un TIA correctement dimensionné. Elle est intéressante si le signal utile est trop faible pour le VEMD4200.

### 10.4 Alternative non retenue : TIA et traitement analogique optique

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

L'OPT4060 est retenu parce qu'il est rapide, colorimétrique, compatible 3,3 V
et disponible sans recourir à une FIFO ni à une alimentation 1,8 V.

## 11. Acquisition audio continue sur le STM32G491

### 11.1 Ne jamais stocker toute la mission

Pour un microphone à 24 kéch/s et 16 bits :

\[
24000 \times 2 = 48000\ \text{octets/s}
\]

Ce débit de 48 ko/s n'est pas un besoin de RAM : les échantillons traversent
successivement le même tampon circulaire.

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

Le rôle ne possède qu'un seul tableau de 1024 `uint16_t`. Chaque moitié est
traitée puis immédiatement réutilisée par le DMA ; aucun appariement stéréo ni
historique brut permanent n'est nécessaire.

### 11.3 Budget RAM mesuré

Le build `-Og` donne les tailles effectives suivantes :

| Élément alloué lorsque le rôle démarre | Taille |
|---|---:|
| `ImavAudioState`, buffer DMA mono inclus | 2 652 octets |
| `ImavLightRange`, contexte ST, buffers I2C et fenêtres optiques inclus | 14 568 octets |
| Pile utile du thread audio | 1 536 octets |
| Pile utile du thread lumière/ToF | 2 048 octets |

Il n'existe pas d'historique audio brut en compétition. Une éventuelle capture
de banc devra être ajoutée explicitement et rester désactivée pendant la
mission.

### 11.4 État mémoire réel de la branche

Les 112 Kio sont répartis entre SRAM1, SRAM2 et CCM. Le projet réserve
notamment un heap standard de 20 Kio, un heap DMA de 12 Kio et le pool mémoire
UAVCAN.

Le rôle lui-même n'est construit que lorsque `ROLE.imav.beacon` est actif.
`ImavAudioState` est alors obtenu par `malloc_dma()`, `ImavLightRange` par
`malloc_m()`, et les deux espaces de travail par `chThdCreateFromHeap()`.
Aucun gros buffer mutable IMAV n'occupe donc la RAM lorsque le rôle est
désactivé. Seuls deux pointeurs de contexte de 4 octets subsistent pour les
trampolines C.

Le portage local du VL53L4CX réutilise le tampon `tofTx` de l'objet dynamique.
Le `_I2CBuffer[256]` du portage générique ST n'est pas lié : la `.bss`
applicative baisse de 35 624 à 35 368 octets et le linker rend ces 256 octets
au heap. Le sous-module officiel ST demeure inchangé.

Le firmware `-Og` synchronisé avec cette note contient 276 472 octets de texte,
3 084 octets de données et 100 056 octets de BSS, heaps réservés inclus. Les
échecs d'allocation sont signalés proprement au démarrage du rôle.

### 11.5 Suréchantillonnage matériel ADC

Le STM32G491 peut accumuler plusieurs conversions dans l'ADC et ne produire qu'un seul résultat par événement de timer. **Le suréchantillonnage n'augmente donc ni le nombre d'échantillons DMA, ni la taille des buffers, ni le débit du DSP.** À 24 kéch/s, la voie reste un flux de 24 000 mots de 16 bits par seconde.

Sous ChibiOS 21.11 :

- le driver ADCv3 active automatiquement la prise en charge du suréchantillonnage sur STM32G4 ;
- `ADCConversionGroup` possède déjà le champ `.cfgr2` ;
- `STM32_ADC_COMPACT_SAMPLES` vaut `FALSE` dans ce projet, donc `adcsample_t` est un `uint16_t` capable de transporter un résultat 13 ou 14 bits ;
- ADC2 est utilisé en mode indépendant et son groupe écrit son propre `CFGR2` ;
- le mode ADC dual reste désactivé.

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

Configuration illustrative du `ADCConversionGroup` audio :

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

`ROVSM` est sans effet utile dans cette architecture puisqu'aucune conversion injectée n'interrompt l'audio ; il peut rester à zéro.

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

Le code DSP ne suppose ni une pleine échelle de 4095 ni un point milieu fixe à
2048 : il retire la moyenne du bloc du microphone. La pleine échelle vaut
environ 8190 en x4/shift1. Les conversions VIN/température/VREF restent en
12 bits sur ADC1, dont le groupe utilise `.cfgr2 = 0`.

Le burst de quatre sous-conversions réalise aussi une petite moyenne temporelle.
Son atténuation dans la bande 2,0–3,0 kHz reste faible, mais il ne remplace pas
un vrai filtre anti-repliement analogique.

ADC2 et TIM6 respectent les contraintes de l'[errata STM32G491 ES0523](https://www.st.com/resource/en/errata_sheet/es0523-stm32g491xx4a1xx-device-errata-stmicroelectronics.pdf), section 2.6.9. Avec l'horloge ADC synchrone actuelle `AHB/4`, le contournement ST consiste à :

- déclencher ADC2 par TIM6 ;
- donner à TIM6 le même rapport de prescaler `/4`, ou un multiple entier.

Pour TIM6 cadencé à 170 MHz, configurer le GPT à 42,5 MHz force donc `PSC = 3`. Un intervalle de 1771 tops fournit environ 23 997,74 Hz, soit seulement -94 ppm par rapport à 24 kHz. Le DSP doit employer cette fréquence réelle dans ses coefficients ou bins Goertzel.

Les sections 2.6.7 et 2.6.8 du même errata imposent aussi de se méfier du
premier résultat après un arrêt logiciel. Chaque balayage ADC1 commence donc
par une conversion VREF factice. Au démarrage ou après une reprise d'ADC2, le
premier demi-buffer audio est jeté et le DSP reçoit un marqueur de
discontinuité.

## 12. Traitement numérique audio

Le STM32G491 possède un Cortex-M4F à 170 MHz, des instructions DSP, une FPU,
CORDIC et FMAC. À 24 kéch/s sur une voie, la charge est faible comparée à la
capacité CPU.

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

La préalarme proche de 2 Hz et l'alarme complète proche de 3 Hz sont toutes
deux valides. La cadence optique reste néanmoins à confirmer sur l'exemplaire
de compétition.

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

## 16. Messages DroneCAN

Le firmware diffuse systématiquement à 5 Hz des
`uavcan.protocol.debug.KeyValue`. Les trois entrées destinées à la navigation
sont `det`, booléen audio transporté comme 0,0 ou 1,0, `snr`, force de la
dernière salve reconnue en dB au-dessus du plancher adaptatif de la bande
2–3 kHz, et `lit`, score des flashs rouges qualifiés par leur cadence. Elles
sont toujours envoyées. Le paramètre booléen
`role.imav.debug.publish.optional`, faux par défaut et appliqué immédiatement,
ajoute seulement les valeurs de mise au point : audio à 5 Hz et échantillons
optiques dérivés à 5 Hz, plus RGBW lossless au rythme data-ready pour le post-traitement. Cette interface
fonctionne avec DroneCAN sans allouer un nouvel identifiant DSDL.

Un message DSDL dédié pourra remplacer les `KeyValue` après avoir choisi le
namespace et l'identifiant avec l'autopilote. Il regrouperait alors les champs
ci-dessous dans un unique transfert.

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

La fréquence de publication implémentée est de 5 Hz. Les événements de salve
pourront éventuellement être publiés séparément si un timestamp plus précis
devient nécessaire.

Ne pas transmettre le flux audio brut sur CAN pendant la mission. Avec un
microphone à 24 kéch/s et 16 bits, il représenterait environ 48 ko/s hors
encapsulation et consommerait inutilement une part importante du bus classique.

Le Makefile utilise par défaut `CAN_BITRATE=1000`, soit CAN classique à
1 Mbit/s. Le brut mono représente déjà 384 kbit/s avant les en-têtes
CAN/DroneCAN, les réémissions et le reste du trafic.

## 17. Intégration logicielle sous forme de rôle

Nom implémenté : `ImavRole`.

Paramètre d'activation :

```text
ROLE.imav.beacon
```

Paramètres runtime implémentés :

```text
role.imav.audio.band_low_hz
role.imav.light.i2c_address
role.imav.time_of_flight
role.imav.tof.period_ms
role.imav.debug.publish.optional
```

`role.imav.audio.band_low_hz` est désormais implémenté : entier persistant
borné entre 2 000 et 2 600 Hz, avec 2 000 Hz par défaut. Il est lu au démarrage
du rôle ; un redémarrage de la MicroCAN est donc nécessaire après modification.
Une carte qui possède déjà la valeur 2 600 Hz en mémoire persistante la
conservera après mise à jour : il faudra lui écrire explicitement 2 000 Hz et
la redémarrer.
Les autres seuils, la borne haute à 3 000 Hz, la fréquence d'échantillonnage et
le temps de conversion OPT4060 sont pour l'instant des constantes du firmware.

Le rôle :

- hérite de `RoleBase` et utilise le schéma CRTP/trampoline existant ;
- acquiert PA4, ADC2 et TIM6 ;
- démarre I2C1 via l'infrastructure commune pour l'OPT4060 et, en option, le VL53L4CX ;
- alloue son unique état/buffer audio par `malloc_dma()` au démarrage ;
- alloue le contexte capteurs et les deux piles de threads au démarrage ;
- utilise un thread audio pour ADC2 et le DSP ;
- utilise un second thread pour l'OPT4060 et, si demandé, pour sérialiser le VL53L4CX ;
- arme ADC2 avant de démarrer TIM6 ;
- ne réalise presque aucun calcul dans le callback ADC ;
- publie les diagnostics sans bloquer l'acquisition ;
- signale les conflits de ressources, manques de mémoire et capteurs absents ;
- reste entièrement désactivable par paramètre.

Le gestionnaire de ressources doit représenter explicitement :

```text
PA04
ADC_2
TIM_6
```

La surveillance VIN/température/VREF reste continue sur ADC1 et indépendante
du thread audio.

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
- un IM68A130(A) sur carte fille, sans OPAMP, relié à PA4/ADC2_IN17 ;
- PA2/PA3 conservées pour la console de débogage ;
- comparaison ADC sans suréchantillonnage, x4/shift1 et x16/shift2 ;
- OPT4060 et VL53L4CX sur I2C1 à 400 kHz ;
- capture de blocs audio ;
- énergie cumulée Goertzel 2,0–3,0 kHz, cadence seulement diagnostique ;
- détection des flashs à 2 ou 3 Hz et rejet de la LED d'état à 1 Hz ;
- exclusion temporelle entre acquisition couleur et mesure ToF ;
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
- plusieurs réglages de temps d'intégration OPT4060.

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
| Bruit hélices malgré l'absence de gain | Clipping ou masque de la balise | Marge directe micro->ADC, drapeau clipping, rapport bande/global et confirmation spatiale |
| Soleil direct | Saturation optique | OPT4060 vers le sol, plage automatique, couleur, retrait du fond et drapeau de saturation |
| LED occultées | Absence de lumière | Accepter le son seul, passages sous plusieurs angles |
| Alarme sonore défaillante | Absence de son | Accepter la lumière seule, avis MSA |
| Module suspendu en oscillation | Erreur de position/niveau | IMU, vitesse faible, rejeter fenêtres instables |
| Variation de réponse optique pendant la rotation | Cartographie perturbée | Capteur vers le sol, suspension anti-lacet et plusieurs passages |
| Fenêtre DSP trop longue en mouvement | Carte décalée | Horodater chaque salve, ralentir au raffinement |
| RAM saturée par les rôles/heaps | Échec au démarrage | États alloués uniquement au lancement, erreurs d'allocation vérifiées et contrôle `.map` |
| PA4 réutilisée en ADC | Perte de `SPI_PERIPH_CS` et TIM3_CH2 | Configuration mission dédiée et gestion de ressources |
| I2C1 réaffecté par un autre rôle | Perte OPT4060/VL53L4CX | Gestion de ressources et configuration mission dédiée |
| Firmware update pendant acquisition SPI future | Conflit de bus | Arrêter rôle avant update, arbitrage explicite |
| Source lumineuse rouge parasite | Faux positif | Cadence, mouvement spatial, fusion audio |
| Autre alarme sonore | Faux positif | Gabarit spectral/temporel et confirmation spatiale |

## 22. Décision recommandée aujourd'hui

Construire d'abord une petite carte fille :

1. **un IM68A130/IM68A130A analogique**, alimenté et découplé proprement ;
2. aucun OPAMP ; sa sortie déjà polarisée rejoint **PA4/ADC2_IN17** par un
   réseau RC passif ;
3. PA2/PA3 conservées pour la console de débogage ;
4. **un OPT4060** orienté vers le sol sur I2C1 ;
5. **un VL53L4CX** orienté vers le sol sur le même bus ;
6. cloison optique entre les deux capteurs et mécanique protégeant le
   microphone du vent ;
7. points de test sur MIC_ADC et l'interruption optionnelle de l'OPT4060 ;
8. aucune photodiode/TIA, FIFO, alimentation 1,8 V ou translation de niveau sur
   la première révision.

L'implémentation actuelle du rôle MicroCAN utilise :

- ADC2 déclenché par TIM6 à 23 997,74 Hz ;
- un DMA circulaire de 1024 × 16 bits, soit 2 Kio ;
- blocs mono de 512 échantillons ;
- suréchantillonnage matériel x4/shift1 ;
- ADC1 maintenu en surveillance continue parallèle pour VIN/température/VREF ;
- rejet du premier demi-buffer audio au démarrage ou après une reprise sur
  erreur ;
- énergie cumulée entre 2,0 et 3,0 kHz et confirmation sur deux blocs ;
- cadence 2/3 Hz mesurée sans conditionner la validité audio ;
- lecture OPT4060 pilotée par data-ready jusqu'à environ 139 Hz, voie rapide
  sur une fenêtre spectrale finie à 2/3 Hz et détecteur adaptatif lent autour de 3 Hz ;
- one-shot VL53L4CX typiquement toutes les 200 ms, sans mesure de lumière
  simultanée ;
- télémétrie de bring-up et publication de la distance au sol.

Ne passer à un AFE actif, un ADC externe ou à des photodiodes analogiques que si les essais montrent que cette chaîne directe est insuffisante. Cette séquence réduit fortement le risque de construire une carte complexe autour d'hypothèses que le règlement ne spécifie pas.

## 23. Questions ouvertes pour la prochaine session

- Peut-on obtenir un motionSCOUT K-T-R réel avant de dessiner la carte fille ?
- Quelle est la hauteur réelle du capteur suspendu, et non celle du drone ?
- Quelle longueur de suspension et quelle garde au sol sont acceptables ?
- Le drone dispose-t-il déjà d'une caméra capable d'identifier les trois mannequins ?
- Quels rôles MicroCAN seront actifs simultanément pendant cette mission ?
- Quel message DSDL/identifiant utiliser pour publier les scores ?
- Comment synchroniser précisément le timestamp MicroCAN avec la pose du contrôleur de vol ?
- L'OPT4060 peut-il voir les flashs sous plein soleil avec l'orientation réelle ?
- Le microphone direct sans gain conserve-t-il assez de SNR en stationnaire ?
- Le suréchantillonnage x16 apporte-t-il réellement plus que x4 dans le bruit du drone ?
- Quelle valeur RC donne le meilleur compromis établissement/anti-repliement sans charger la sortie des microphones ?
- La mémoire DMA heap disponible dans la configuration mission permet-elle l'allocation du tampon sans réduire d'autres rôles ?
- Quelle période ToF donne le meilleur compromis entre asservissement de
  hauteur et fenêtres optiques aveugles ?
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
- [TI OPT4060](https://www.ti.com/product/OPT4060)
- [Vishay VEMD4200FX01](https://www.vishay.com/en/product/84950/)
- [Vishay BPW34](https://www.vishay.com/en/product/81521/)
- [TI OPA381](https://www.ti.com/product/OPA381)
- [TI OPA320](https://www.ti.com/product/OPA320)

## 25. État de l'implémentation au 1er septembre 2026

La branche active est `imav2026`. Un premier rôle fonctionnel
`ImavRole` est intégré, compilé par `USE_IMAV_ROLE` et activé à l'exécution par
`ROLE.imav.beacon`. Aucun autre rôle n'a été désactivé statiquement.

### 25.1 Acquisition et partage des ressources

- PA4/ADC2_IN17 est acquis avec ADC2 et TIM6 par le gestionnaire de ressources.
- ADC2 utilise une conversion régulière circulaire d'un canal, un DMA de 1024 mots et un callback minimal.
- ADC1 reste exclusivement affecté à la surveillance continue VIN/température/VREF.
- TIM6 fournit le TRGO à `42,5 MHz / 1771 = 23 997,74 Hz` ; son prescaler `/4` respecte le contournement de l'errata avec l'horloge ADC AHB/4.
- Le suréchantillonnage est x4 avec shift 1 : sortie 13 bits, débit DMA inchangé.
- Un watchdog relance l'acquisition si aucun demi-buffer ne progresse pendant 100 ms.
- La surveillance ADC1 et le flux audio ADC2 fonctionnent sans pause ni commutation de groupe.
- Après le démarrage ou une reprise sur erreur, le premier demi-buffer audio est jeté et le suivant porte un marqueur de discontinuité.

PA2 et PA3 restent disponibles pour `DebugTrace` et le shell complet. Sur cette
branche dédiée, PA4 est configurée en entrée analogique dès `halInit` afin de
ne jamais forcer électriquement la sortie du microphone ; le rôle confirme ce
mode avant de démarrer ADC2.

### 25.2 DSP audio actuel

Chaque bloc mono de 512 échantillons est consommé immédiatement, sans historique brut :

1. moyenne, écart absolu moyen, minimum, maximum et clipping ;
2. fenêtre de Hann générée par récurrence, donc sans table RAM ;
3. grille Goertzel fixe de 50 Hz permettant de choisir une borne basse entre
   2,0 et 2,6 kHz ; la valeur par défaut 2,0 kHz utilise les références à
   1,65/1,75 et 3,25/3,40 kHz ainsi qu'une marge de 50 Hz autour de la bande ;
4. puissance moyenne de bande, proéminence obligatoire sur les références,
   rapport en dB entre l'énergie cumulée de bande et l'énergie globale, et
   fréquence dominante indicative ; une simple montée d'énergie large bande
   ne suffit pas à valider un bloc ;
5. plancher lent adaptatif et score continu par voie ;
6. machine `Unarmed/Off/On` avec hystérésis et validation d'une paire de blocs
   spectraux cohérente en niveau et en fréquence ;
7. six horodatages de fronts au maximum et mesure des cadences proches de 2 et
   3 Hz, sans utiliser cette cadence comme condition de validité.

L'état `Unarmed` est réimposé après une discontinuité. Il faut ensuite observer
deux blocs bas. Un front demande deux blocs adjacents valant chacun au moins
0,45, dont la somme atteint 1,20, et dont les fréquences dominantes diffèrent
de moins de 250 Hz. Un bloc inférieur ou égal à 0,30 termine la salve. Deux
fronts acceptés sont séparés d'au moins 250 ms ; un intervalle supérieur à
750 ms réinitialise l'historique au lieu de polluer la cadence suivante. Ces
bornes couvrent les périodes de 333 ms et 500 ms tout en rejetant la
réverbération d'une même salve. Le score est maintenu pendant 750 ms puis
décroît jusqu'à 1 500 ms afin de couvrir les silences de la préalarme et de
l'alarme. Une tonalité continue fortement concentrée dans la bande peut donc
être acceptée : c'est un choix délibéré compte tenu du faible risque de leurre
dans la mission.

Le banc avec enceinte a validé des chirps 2,1–2,5 kHz de 60 à 120 ms. Avec
120 ms, la préalarme est mesurée à 1,98–2,04 Hz et l'alarme complète à
2,96–3,04 Hz. Le bruit blanc continu et des salves de bruit blanc à 3 Hz sont
rejetés. Après trois secondes d'apprentissage d'un bruit blanc continu, un
chirp dont le niveau RMS est supérieur de 6 dB reste détecté à 3 Hz. Ces
mesures qualifient le banc, pas encore le bruit aérodynamique réel du drone.

Les seuils numériques actuels sont volontairement des valeurs de bring-up. Ils doivent être recalés sur des enregistrements de la vraie balise et du drone, en conservant les scores continus plutôt qu'un simple booléen.

### 25.3 OPT4060 et VL53L4CX optionnel

- Un unique OPT4060 orienté vers le sol est recherché aux adresses 0x44 à 0x47 ; `ADDR=GND` et l'adresse 0x44 sont les valeurs nominales.
- Le registre de configuration vaut `0x30B8` en acquisition : plage automatique, 1,8 ms par voie, mode continu et quatre voies séquentielles. Un cycle RGBW typique dure donc 7,2 ms.
- `INT_CFG=3` fait émettre par l'OPT4060 une impulsion active-bas de 1 µs après
  chaque groupe RGBW complet. PA8, ancien signal SRV1, est configuré en
  `OPT4060_INT` avec pull-up et EXTI8. Le thread attend le front descendant avec
  `palWaitLineTimeout`, puis lit les huit registres de résultat par un unique
  transfert burst. Un timeout de 25 ms écrit un avertissement sur le shell
  série et effectue une lecture de secours. Le débit nominal maximal reste
  d'environ 139 Hz. Exposant et mantisse sont
  linéarisés en codes ADC 26 bits. Le CRC matériel protège les 20 bits de
  mantisse, les 4 bits d'exposant et les 4 bits du compteur ; une discordance
  invalide tout le groupe. Les compteurs restent ensuite vérifiés afin de
  détecter une donnée périmée ou un capteur bloqué. Le registre de statut
  `0x0C` n'est lu que si un canal atteint l'exposant maximal 6 ou la mantisse
  maximale ; la transaction I2C supplémentaire disparaît en régime nominal.
- Les coefficients TI `R=2,4×CH0`, `G=CH1` et `B=1,3×CH2` sont appliqués avant de calculer la dominance rouge de la variation. Le score instantané combine amplitude absolue, variation relative, élévation au-dessus du bruit et chromaticité rouge.
- La voie rapide conserve exactement la dernière seconde de RGB dans un
  anneau de 144 échantillons. Des sommes glissantes évaluent cinq couples
  fondamental/H2 sans seuil de front ni niveau absolu. Cohérence, couleur,
  rapport H2/H1 et phase relative forment `lfs`; les sommes sont reconstruites
  périodiquement pour borner les erreurs d'arrondi en vol long. Cette voie est
  exposée par `lhz/lcs/lfs/lps/lpc/lon/lof/lts/lpt`.
- La voie lente de `lit` provient d'une DFT glissante sur le contraste rouge
  `R - (G+B)/2`. Chaque motif activé possède une banque fondamentale dense par
  pas de 0,1 Hz, centrée sur la fréquence déduite de ses temps haut/bas et
  large de ±0,8 Hz. Les cinq candidats centraux, à ±0,2 Hz, possèdent chacun
  leur voie à la deuxième harmonique ; le meilleur est comparé à la médiane
  des bins fondamentaux non adjacents.
  Cette DFT horodatée joue le rôle de voies Goertzel parallèles tout en
  acceptant les petites irrégularités de cadence de l'interruption data-ready.
  Le fond continu est retiré, l'intégration exponentielle a une constante de
  temps voisine de 10 s, et le support monte progressivement pendant les dix
  premières secondes. Les coefficients exponentiels sont calculés à partir
  des horodatages, de sorte que le débit data-ready et ses éventuelles
  irrégularités ne changent pas les constantes de temps. Le score exige
  simultanément une proéminence locale
  du pic et une cohérence périodique ; la couleur rouge est un qualificatif
  souple pour tolérer les réflexions du sol. Le rapport H2/H1 attendu vaut
  `abs(cos(pi*duty_cycle))`; son accord, la phase de H2 par rapport au carré
  complexe de H1 et la visibilité de H2 forment `lhs`. Une fondamentale
  dépourvue de cette signature asymétrique reste exploitable à demi-confiance,
  mais ne peut plus porter seule la voie spectrale à 1.
- `role.imav.light.high_ms=100`, `steady_low_ms=233` et
  `beginning_low_ms=400` décrivent la forme nominale sans recompilation. Avec
  `role.imav.light.beginning_pattern=false`, valeur de concours par défaut,
  seule la banque du régime établi est calculée ; `true` autorise en plus la
  phase de démarrage proche de 2 Hz et retient la meilleure banque. Ces
  paramètres sont lus au lancement du rôle et demandent donc un redémarrage.
  La voie rapide en déduit également les durées `lon/lof` à partir de H2/H1.
- En acquisition, le score nominal `lit` prend le maximum de la voie rapide
  `lfs` et de la voie spectrale lente `lsc`. Lorsque `lfs` atteint 0,55, un
  mode de suivi de proximité est mémorisé : `lfs` pondère alors la contribution
  lente afin qu'elle ne masque pas l'éloignement après le survol. Lorsque la
  fenêtre rapide retombe sous 0,15, l'état lent devenu périmé est effacé et une
  nouvelle acquisition longue portée repart proprement. Dans les rejeux, `lit`
  passe sous 0,30 entre 0,16 et 0,50 s après l'extinction ; aucun échantillon
  rapide ne survit plus d'une seconde.
- Cette normalisation locale ne dépend pas du niveau lumineux absolu ni du
  gain choisi automatiquement par l'OPT4060. Elle vise notamment le signal à
  5 m, environ dix fois plus faible que celui mesuré à 50 cm lorsque la balise
  d'essai est dix fois plus puissante. Elle ne dispense pas d'une capture
  négative en extérieur : ombres mobiles, hélices et reflets proches de 3 Hz
  doivent encore fixer les seuils définitifs.
- `role.imav.time_of_flight` vaut `false` par défaut. Dans cet état, le firmware n'accède jamais à l'adresse 0x29, ne lance aucune initialisation ou relance VL53L4CX et ne suspend jamais l'OPT4060.
- Lorsque cette option est activée, le cœur officiel ST du VL53L4CX reste inchangé dans le sous-module `third_party/x-cube-tof1`, épinglé sur X-CUBE-TOF1 v3.4.3 (cœur VL53LX 1.2.13). Le portage générique `vl53lx_platform.c` est exclu et remplacé par le portage local ChibiOS, qui utilise les callbacks I2C/temps du projet et le buffer `tofTx` alloué avec le rôle.
- Il fonctionne en profil longue distance, budget de 30 ms et one-shot asynchrone borné à 100 ms. Le résultat valide le plus proche est publié.
- Avant chaque one-shot ou réinitialisation ToF, le mode de l'OPT4060 passe à `Power-down` par une écriture I²C acquittée. La mesure ST est effectuée, puis le mode continu est rétabli ; aucun résultat optique de cette fenêtre n'est utilisé.
- La période nominale vaut 200 ms et est paramétrable. Un petit jitter déterministe déplace la phase des trous afin de ne pas masquer systématiquement une balise périodique.
- I2C standard 100 kHz et Fast mode 400 kHz sont acceptés ; 400 kHz reste recommandé. Le Fast-mode Plus 1 MHz est refusé car l'OPT4060 passe directement du Fast mode au protocole High-Speed 2,6 MHz, non activé ici.
- Un NACK d'un capteur absent ne réinitialise pas le bus partagé. Seuls un timeout ou une faute électrique/protocolaire déclenchent la récupération sérialisée.

Le test H7 du 1er septembre 2026, antérieur au passage sur l'interruption PA8
et à l'activation de la voie rapide dans `lit`,
contient 10 s éteintes, la montée réelle de
l'alarme puis plus de 30 s au régime final. Sur 64,3 s, les 6 335 groupes
optiques sont continus, sans trou de compteur, surcharge ni erreur I2C. Avec
la voie spectrale seule de ce firmware, `lit` reste exactement nul avant
l'établissement de la cadence finale. Après son
apparition, il franchit 0,5 à 30,632 s, 0,8 à 31,038 s et atteint 1 à
31,443 s. En régime établi, le pic reste verrouillé à 3,0 Hz, sa proéminence
vaut 24,1 à 31,0 dB, sa cohérence moyenne 0,65 et sa fraction rouge périodique
0,76. La capture locale, non versionnée en raison de son volume, est
`tools/imav_monitor/captures/imav_spectral_ramp_20260901T_light.csv`, SHA-256
`eb982de2e15696d2e45050085f0e4cbeabfd64434164972ac605ff6749648cc7`.

Le test attaque/relâchement du 2 septembre 2026 utilise l'interruption PA8 et
la fusion rapide/lente du point de retour `dd2b36a`, avant qualification des
durées et harmoniques. Sur 41,8 s, 4 522 groupes RGBW sont reçus sans perte,
surcharge ni erreur I2C. Pendant la préalarme à 2 Hz, `lit` dépasse 0,7 au
troisième flash accepté, 1,00 s après le premier ; le même nombre de flashs
demande environ 0,67 s au régime 3 Hz rencontré en approche. À l'extinction,
le dernier front est publié à 23,358 s, `lit` passe sous 0,3 à 24,145 s et
atteint zéro à 24,358 s : la décroissance observée sur CAN prend donc environ
0,79 s jusqu'au seuil bas et 1,00 s jusqu'à zéro. La capture locale non
versionnée est
`tools/imav_monitor/captures/imav_fast_attack_release_20260902.csv`, SHA-256
`178581350bdc683734569a73b93648c945af7aee2b34e6bd72793d1cc69ab494`.

Un premier rejeu hors cible de cette capture avec les nouveaux paramètres par
défaut rejetait la phase 2 Hz et atteignait 0,994 à 3 Hz. Le test suivant sur
cible a cependant révélé une faiblesse que cette capture ne contenait pas :
avec un fond lumineux différent, `lis` restait souvent entre 0,4 et 0,6 au
niveau bas. Le seuil descendant fixe à 0,25 n'était plus franchi, l'automate
restait bloqué dans l'impulsion et `lfs` demeurait nul. La DFT lente restait
alors proche de 1 plus de vingt secondes après extinction puisqu'elle n'avait
jamais été placée sous le contrôle de fraîcheur rapide. Cette implémentation
temporelle par seuils n'est donc pas retenue en l'état.

Le candidat offline suivant supprime entièrement les seuils de fronts. Une
fenêtre glissante finie de 1 s évalue cinq fréquences autour du motif établi et
leur H2 associée sur les échantillons RGB bruts. Le score combine cohérence du
fondamental, rapport H2/H1, phase H2 par rapport à H1² et couleur périodique.
Sur six captures totalisant 62 514 groupes RGBW, les 3 204 fenêtres éteintes
restent sous 0,038, les 1 069 fenêtres du début 2 Hz rejeté restent sous 0,147
et les 5 279 fenêtres établies restent au-dessus de 0,864. Les transitions 3
Hz franchissent 0,55 en 0,25 à 0,56 s et les extinctions passent sous 0,30 en 0,16
à 0,50 s. Lorsque la banque de démarrage est autorisée, elle est sélectionnée
pour 100 % des fenêtres 2 Hz et la banque établie pour 100 % des fenêtres 3
Hz. Le rejeu reproductible est fourni par
`tools/imav_monitor/offline_light_detector.py` et ses annotations par
`light_capture_segments.json`. Le même calcul à sommes glissantes est
maintenant reporté dans l'arbre de travail du firmware, mais cette version
n'est pas encore validée sur la cible.

Sur la capture longue du 1er septembre, H2/H1 vaut 0,807 à 2 Hz pour 0,809
attendu, puis 0,577 à 3 Hz pour 0,587 attendu ; les erreurs de phase
respectives restent 0,074 et 0,014 rad. Ces essais valident l'exploitation de
l'asymétrie sur le banc, mais le réglage final attend encore une acquisition
avec la vraie balise.

Une simulation prudente réinjecte le fondamental mesuré à 10 % de son
amplitude dans le résidu de cette même séquence, conformément à l'hypothèse
5 m / puissance lumineuse ×10. Le score établi vaut environ 0,57 avec le bruit
mesuré, 0,37 avec ses variations rapides multipliées par deux et 0,27 avec un
facteur quatre. Ces valeurs justifient une sortie continue destinée à la
cartographie spatiale, pas un seuil booléen universel. Le résidu provient en
outre d'un banc intérieur : seule une capture négative extérieure pourra
qualifier le taux de faux positifs réel.

Lorsque le ToF est activé, la distance est publiée à chaque mesure dans `uavcan.equipment.range_sensor.Measurement` (`sensor_id=0`, type LIDAR, champ de vue 18°). L'orientation de corps est laissée indéfinie car le module est suspendu ; le contrôleur de vol configure lui-même l'orientation du télémètre vers le bas.

La lumière et le son restent deux preuves indépendantes. Il n'y a pas de condition rigide `son ET lumière`, notamment à cause de l'avis MSA 2026 sur des alarmes sonores potentiellement défaillantes.

### 25.4 RAM, threads et télémétrie

Mesures contrôlées sur le build `-Og` :

- `sizeof(ImavAudioState) = 2 652` octets sur le heap DMA de 12 288 octets ; ce total contient le buffer audio mono de 2 048 octets ;
- `sizeof(ImavLightRange) = 14 568` octets sur le heap standard de 20 480 octets ; ce total contient le contexte officiel ST de 9 480 octets, les tampons I2C, les banques DFT lentes et l'anneau rapide ;
- thread audio : pile utile configurée à 1 536 octets ;
- thread capteurs : pile utile configurée à 2 048 octets ;
- firmware expérimental : 269 760 octets de texte et 100 096 octets de BSS,
  heaps réservés inclus ;
- le shell de 2 000 octets n'est pas alloué en mode IMAV.

Le portage matériel local du VL53L4CX réutilise le tampon `tofTx` contenu dans
`ImavLightRange`. Le `_I2CBuffer[256]` du portage générique ST n'est donc plus
lié au firmware, sans modifier le sous-module officiel. La section `.bss`
applicative mesurée vaut 35 424 octets ; les 256 octets du tampon générique
restent rendus au heap par le linker. Le total BSS synthétique vaut
100 048 octets parce qu'il inclut ce heap. Lorsque le rôle est
compilé mais désactivé, il ne reste en RAM que les deux pointeurs de trampoline
de 4 octets ; tous ses buffers et états sont alloués à son lancement.

La télémétrie fonctionnelle `uavcan.protocol.debug.KeyValue` est diffusée à
5 Hz lorsque le rôle IMAV est actif.

Le contrat opérationnel et la séparation entre messages de mission et
instrumentation de banc sont détaillés dans [can_msg.md](can_msg.md).

Les clés publiées sont :

| Clé | Valeur |
|---|---|
| `det` | détection audio avec hystérésis, exactement 0,0 ou 1,0 ; entrée principale de l'autopilote |
| `snr` | pic de la puissance excédentaire de la salve rapportée en dB au plancher adaptatif dans la bande 2–3 kHz ; maintenu entre les bips puis ramené à zéro lorsque le signal devient périmé |
| `a0` | optionnel : score spectral instantané du microphone |
| `p0` | optionnel : amplitude RMS estimée de la tonalité dominante, en comptes ADC 13 bits |
| `sdb` | optionnel : rapport instantané en dB entre l'énergie de la bande balise et l'énergie globale de la fenêtre ; ce n'est pas le SNR moteur |
| `aud` | optionnel : score de signature spectrale audio, maintenu pendant les silences |
| `frq` | optionnel : fréquence dominante en hertz |
| `cad` | optionnel : cadence de salves estimée en hertz |
| `lit` | score lumineux combiné : acquisition longue portée, puis suivi rapide des flashs rouges à 2/3 Hz après verrouillage de proximité ; entre 0 et 1 et zéro après 200 ms sans nouvel échantillon |
| `lrd` | optionnel, jusqu'à environ 139 Hz : code ADC rouge linéarisé |
| `lgn` | optionnel, jusqu'à environ 139 Hz : code ADC vert linéarisé |
| `lbl` | optionnel, jusqu'à environ 139 Hz : code ADC bleu linéarisé |
| `lwh` | optionnel, jusqu'à environ 139 Hz : code ADC large bande linéarisé |
| `lov` | optionnel, jusqu'à environ 139 Hz : indicateur de surcharge de l'échantillon, 0 ou 1 |
| `lct` | optionnel, jusqu'à environ 139 Hz : compteur d'échantillons permettant de reconstruire les groupes et les pertes |
| `ltu` | optionnel, jusqu'à environ 139 Hz : horodatage matériel en microsecondes modulo 2^24, exact en float et déroulable toutes les 16,78 s |
| `lrr` | optionnel 5 Hz : fraction rouge de l'excursion RGB positive, entre 0 et 1 |
| `lac` | optionnel 5 Hz : excursion rouge positive rapportée au fond lumineux |
| `lis` | optionnel 5 Hz : score instantané d'une impulsion rouge, entre 0 et 1 |
| `lhz` | optionnel 5 Hz : fréquence du meilleur fondamental de la fenêtre rapide |
| `lcs` | optionnel 5 Hz : cohérence du fondamental rapide, entre 0 et 1 |
| `lfs` | optionnel 5 Hz : score spectral de proximité sur la dernière seconde |
| `lps` | optionnel 5 Hz : amplitude fondamentale rapide rapportée au rouge moyen |
| `lpc` | optionnel 5 Hz : compteur cumulé des verrouillages de la voie rapide |
| `lsa` | optionnel 5 Hz : compteur cumulé d'épisodes de saturation OPT4060 |
| `ler` | optionnel 5 Hz : compteur cumulé d'erreurs de lecture OPT4060 |
| `lgp` | optionnel 5 Hz : compteur cumulé de trous d'échantillonnage lumineux |
| `lsc` | optionnel 5 Hz : score spectral brut utilisé par `lit`, sans le contrôle de fraîcheur CAN |
| `lsn` | optionnel 5 Hz : proéminence du meilleur bin autour du motif configuré, en dB par rapport à la médiane spectrale locale |
| `lco` | optionnel 5 Hz : cohérence du fondamental périodique dans le contraste rouge, entre 0 et 1 |
| `lrf` | optionnel 5 Hz : fraction rouge de la composante périodique complexe, entre 0 et 1 |
| `lfq` | optionnel 5 Hz : fréquence du meilleur bin spectral, en hertz |
| `lhr` | optionnel 5 Hz : rapport d'amplitude H2/H1 du meilleur bin spectral |
| `lhs` | optionnel 5 Hz : score de forme harmonique attendu pour le créneau asymétrique |
| `lon` / `lof` | optionnel 5 Hz : durées haute et basse estimées par H2/H1, en ms |
| `lts` | optionnel 5 Hz : score de forme harmonique de la fenêtre rapide |
| `lpt` | optionnel 5 Hz : motif sélectionné, 0 aucun, 1 démarrage, 2 établi |
| `rng` | optionnel avec ToF : distance en mètres, -1 si la dernière mesure n'est pas valide |
| `rsg` | optionnel avec ToF : signal VL53L4CX en kcps/SPAD |

Les clés font trois caractères afin que chaque message tienne dans une seule
trame CAN classique. Le mode nominal produit 15 trames/s (`det`, `snr` et
`lit` à 5 Hz). Le débogage optionnel produit en plus 140 trames/s dérivées et
jusqu'à environ 973 trames/s optiques brutes, soit 1 128 trames/s sans ToF et
1 138 trames/s avec ToF. Cette charge d'essai représente approximativement
16 % du CAN
classique à 1 Mbit/s ; toutes ces trames gardent une priorité basse. Le
plancher de `snr` apprend les blocs non reconnus comme
balise : il suit le bruit blanc continu des moteurs mais n'absorbe pas les
salves périodiques. Chaque pic de salve est lissé avec un coefficient de 0,5 ;
la valeur reste stable dans les silences de 333/500 ms, puis suit la même
décroissance de fraîcheur que `aud` après 750 ms sans tonalité.

La réception SocketCAN avec PyDroneCAN confirme cinq groupes nominaux par
seconde, espacés alternativement d'environ 192 et 213 ms à cause des blocs DSP
de 21,33 ms. Avec l'option à `false`, seuls `det`, `snr` et `lit` sont
observés. Son passage à `true` ajoute immédiatement les vingt-huit clés dérivées à
5 Hz, les sept clés optiques lossless au rythme data-ready et, le cas échéant, deux clés
ToF à 5 Hz, sans
redémarrage. Sur le banc, trois fichiers séparés par pas de 6 dBFS donnent des
plateaux `snr` d'environ 12,0, 17,6 et 23,7 dB : les écarts mesurés de 5,6 et
6,1 dB confirment que la grandeur est utilisable comme force relative pour la
cartographie spatiale. Ces valeurs absolues dépendent toutefois du bruit de
fond, du haut-parleur, du microphone et de la géométrie du test.

### 25.5 Validation encore nécessaire

- mesurer le temps CPU maximum du DSP sur cible en `-Og` puis en `RELEASE=fast` ;
- injecter des sinus/salves connus sur PA4 et vérifier fréquence, cadence,
  clipping et reprise après une erreur ADC2 ;
- enregistrer la vraie forme temporelle du motionSCOUT et ajuster les seuils/profils de cadence ;
- mesurer la largeur réelle des flashs et vérifier qu'un allumage produit plusieurs cycles RGBW complets ;
- tester l'OPT4060 au soleil, à l'ombre, devant des LED parasites et avec l'ouverture mécanique définitive ;
- observer les écritures `Power-down` et l'émission 940 nm pour confirmer sur cible qu'il n'existe aucun recouvrement lumière/ToF ;
- qualifier le VL53L4CX vers 1 m au-dessus de sols clairs et sombres, en extérieur, avec le verre de protection et le balancement du fil ;
- configurer l'orientation vers le bas dans le contrôleur de vol et valider la consommation de `uavcan.equipment.range_sensor.Measurement` ;
- décider la fusion et les durées de confirmation monomodales/bimodales après ces essais ;
- surveiller les heaps dans la configuration exacte de mission avant d'envisager une désactivation statique d'autres rôles.
