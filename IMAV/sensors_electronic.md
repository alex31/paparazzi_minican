# Carte capteurs IMAV — document de travail

> Version de discussion du 22 juillet 2026
>
> Cible : MicroCAN v5 / STM32G491
>
> Statut : proposition pour le premier prototype, à commenter en équipe

## 1. Balise cible et cadre IMAV 2026

### 1.1 Ce que fixe le règlement

La source de référence est le
[règlement IMAV 2026 v2.1](Rulebook_IMAV2026_V2.1.pdf), sections 5.4.7 et
5.4.8, page 34.

| Sujet | Donnée utile pour le projet |
|---|---|
| Balise annoncée | **MSA motionSCOUT K-T-R** |
| Signal sonore | bande **2,6 à 3,0 kHz** ; alarme annoncée à **95 dB à 3 m** |
| Signal lumineux | **deux LED très lumineuses** ; le règlement ne précise ni leur couleur ni leur chronogramme |
| Zone de recherche | un point GPS donné le jour de l'épreuve ; trois mannequins dans un rayon de 25 m |
| Cible | un seul des trois mannequins porte la balise |
| Dépose | colis déposé ou largué à moins de 2 m d'altitude ; distance mesurée du nombril au colis, limitée à 50–300 cm pour le calcul |
| Enjeu de la détection | la partie principale du score est multipliée par deux près du bon mannequin |
| Autonomie | facteur 1 si tout est autonome, 0,7 avec calcul hors bord et 0,4 en cas d'action manuelle |

Le texte conserve la mention « puissance sonore à définir » tout en donnant
95 dB à 3 m entre parenthèses. Cette valeur est donc notre base de travail,
mais elle mérite une confirmation de l'organisation.

### 1.2 Comportement de la motionSCOUT K-T-R

Le
[manuel MSA 10251777/01](OPM_motionSCOUT_10251777_01_Book.pdf) couvre toute la
famille motionSCOUT et pas uniquement la balise de la compétition. Son tableau
de commande associe explicitement la version motionSCOUT `K` avec les options
`T-R` à la référence **10088478**. C'est donc bien la variante visée par le
règlement : version à clé (`K`), avec capteur de température (`T`) et
réinitialisation manuelle de l'alarme (`R`).

D'après ce manuel :

1. le retrait de la clé active l'appareil et lance un autotest sonore et
   lumineux d'environ 2 s ;
2. après 25 s sans mouvement, une préalarme dure environ 15 s ; elle émet deux
   signaux sonores par seconde à niveau croissant et s'annule dès que la balise
   bouge ;
3. si l'immobilité continue, l'alarme complète démarre, soit environ 40 s après
   le dernier mouvement ; elle peut aussi être déclenchée immédiatement par le
   bouton d'alarme ;
4. l'alarme complète émet trois signaux sonores par seconde au niveau maximal
   et active les deux LED rouges d'alarme ; le manuel ne donne pas la cadence
   de ces LED ;
5. sur cette version `R`, deux appuis sur le bouton d'alarme en moins d'une
   seconde réinitialisent l'alarme ;
6. pour éteindre la version à clé, il faut réinsérer la clé puis maintenir le
   bouton ON/OFF pendant au moins 4 s.

La balise possède **deux LED rouges d'alarme** et une **LED bicolore d'état**
distincte. Cette dernière clignote à 1 Hz en vert lorsque la batterie est
pleine. En fin de batterie, une brève alerte sonore est suivie de son
clignotement rouge à 1 Hz ; le manuel annonce alors environ 1 h d'autonomie en
alarme complète. Cette LED est uniquement un témoin destiné à l'utilisateur :
elle ne fait pas partie du dispositif d'alerte et ne doit jamais contribuer à
la décision de détection. Le signal optique utile provient exclusivement des
deux LED rouges d'alarme. Le fabricant ne donne pas leur cadence, la durée de
leurs éclairs ou leur intensité.

La brochure MSA indique une bande de 2,6 à 3,0 kHz, comme le règlement, tandis
que le manuel donne une bande plus large de 2,0 à 3,0 kHz. Nous dimensionnons
d'abord la détection sur la bande réglementaire, puis nous mesurerons le spectre
de l'exemplaire réellement utilisé.

Caractéristiques secondaires utiles pour préparer les essais :

- dimensions : 100 × 75 × 45 mm ; masse : 230 g avec piles ;
- alimentation : deux piles AA LR6 ; autonomie annoncée supérieure à 200 h en
  marche et 10 h en alarme complète ;
- indices IP66/IP67 ; température de fonctionnement : -20 à +50 °C ;
- au-dessus de 80 °C interne, l'alarme thermique émet un son bitonal toutes les
  2,5 s jusqu'au refroidissement ; ce cas est peu probable pendant l'épreuve.

Sources fabricant :
[brochure motionSCOUT](https://s7d9.scene7.com/is/content/minesafetyappliances/motionSCOUT%20Bulletin%20-%20FR),
[manuel MSA 10251777/01](OPM_motionSCOUT_10251777_01_Book.pdf)
et
[avis de service de février 2026](https://assetlibrary.msasafety.com/m/1daafc2b15a7c2a5/original/Avis-de-service-Dispositif-MSA-motionSCOUT-PASS-Fevrier-2026.pdf).

### 1.3 Points à confirmer sur la balise de compétition

Le règlement et les documents MSA ne définissent pas :

- l'orientation et la fixation de la balise sur le mannequin ;
- la largeur, la cadence et la puissance optique des éclairs, leur diagramme
  angulaire, ni leur synchronisation éventuelle avec le son ;
- la forme d'onde sonore, ses harmoniques, ses tolérances et les conditions de
  mesure des 95 dB ;
- l'état des piles et les conditions réelles de soleil, de bruit, de vent et
  d'occultation pendant la mission ;
- la référence et la date de fabrication de l'exemplaire que l'organisation
  installera sur le mannequin.

Il faut donc demander si possible une balise identique pour les essais, ou au
minimum enregistrer l'exemplaire de compétition sous plusieurs angles et à
plusieurs distances.

Un avis de service MSA de février 2026 inclut la référence 10088478 fabriquée
entre juin 2024 et décembre 2025 : sur certaines unités, l'alarme sonore peut
cesser alors que l'alarme visuelle continue de clignoter. L'organisation devra
confirmer la date ou le remplacement de son exemplaire et effectuer un test de
bon fonctionnement avant l'épreuve.

### 1.4 Conséquences pour notre détecteur

- La voie audio utilisera l'énergie dans la bande 2,6–3,0 kHz, mais aussi la
  cadence caractéristique de trois signaux par seconde en alarme complète.
- La voie optique cherchera des variations rapides et répétées sans supposer
  une durée d'éclair connue.
- La LED d'état à 1 Hz n'est pas un indice de présence de la balise. Son
  éventuelle contribution au signal du capteur est un parasite qui ne doit pas
  déclencher la détection.
- L'alarme thermique bitonale à 0,4 Hz n'est pas non plus le signal recherché.
- Les détections sonore et lumineuse resteront indépendantes : leur accord
  augmentera la confiance, mais un `ET` strict risquerait de rejeter une vraie
  balise.
- L'exclusion temporelle entre mesure de lumière et mesure ToF ne sera validée
  qu'après mesure de la répétition réelle des éclairs et de la fenêtre aveugle.
- Comme une seule cible doit être distinguée parmi trois mannequins, la décision
  devra s'appuyer sur plusieurs mesures cohérentes dans l'espace, pas sur une
  détection instantanée isolée.

## 2. Contexte

Le module sera suspendu sous le drone par un fil de 2 à 3 m. Le drone volera à
environ 3 à 4 m et la carte capteurs se trouvera approximativement à 1 m du sol.
Elle doit aider à retrouver un mannequin couché équipé d'une balise sonore et
lumineuse.

Le signal utile venant du sol, la proposition la plus simple est d'utiliser un
seul capteur par fonction, tous orientés ou ouverts vers le bas :

- un microphone pour la balise sonore ;
- un capteur rapide pour les flashs ;
- un télémètre pour contrôler la hauteur du module au-dessus du sol.

Ce document présente la solution actuellement envisagée. Il ne constitue pas
encore une spécification de fabrication définitive.

## 3. Proposition actuelle

| Fonction | Composant retenu | Interface | Motivation principale |
|---|---|---|---|
| Son | Infineon IM68A130A | PA3 / ADC1_IN4 | microphone analogique sensible, un seul canal ADC |
| Flash | ams OSRAM TCS34103M | I²C, adresse 0x39 | acquisition continue rapide et FIFO 512 octets |
| Distance au sol | ST VL53L4CXV0DH/1 | I²C, adresse 0x29 | portée suffisante autour de 1 m et pilote officiel ST |

Décisions déjà raisonnablement établies :

- un seul microphone et un seul capteur de flash ;
- TCS3410 et VL53L4CX orientés vers le sol ;
- microphone relié directement à l'ADC, sans amplificateur externe ;
- TCS3410 et VL53L4CX sur le même bus I²C ;
- bus à 400 kHz pour le premier prototype ;
- pull-up I²C de 2,2 kΩ déjà présentes sur la MicroCAN ;
- aucune mesure de lumière pendant une mesure ToF ;
- alimentation générale 3,3 V, avec un petit rail 1,8 V uniquement pour le
  cœur du TCS3410 ;
- aucun traducteur de niveau I²C : les entrées/sorties du TCS3410 acceptent les
  pull-up à 3,3 V.

Le TCS34103M est en phase « Last Time Buy », mais cela n'est pas considéré comme
bloquant pour une compétition ponctuelle. Il faut simplement acheter assez de
pièces pour les prototypes et les reprises de montage.

## 4. Utilisation envisagée des capteurs

### 4.1 Microphone

L'IM68A130A fournit déjà un signal analogique amplifié et polarisé autour de
1,3 V. Il peut donc être relié directement à l'ADC 0–3,3 V :

- pas de condensateur de liaison ;
- pas de pont de polarisation ;
- pas d'OPAMP ;
- un simple filtrage passif de l'alimentation et une petite résistance série
  devant l'ADC.

L'indicateur sonore proposé pour discriminer la balise du bruit des moteurs est
le rapport :

`10 × log10(énergie dans les bandes de la balise / énergie audio totale)`

Il faudra lui ajouter un seuil minimal d'énergie absolue pour éviter qu'un
rapport élevé calculé sur du silence soit interprété comme une détection. Les
largeurs de bandes et les seuils devront être déterminés à partir
d'enregistrements réels sous le drone. La présence d'une enveloppe répétée à
environ 3 Hz permettra de renforcer la détection de l'alarme complète ; la
préalarme à 2 Hz ne doit pas être notre état nominal de recherche.

### 4.2 Flash

Le TCS3410 possède des canaux RGB, Clear et un canal Flicker très sensible. Le
logiciel actuel utilise le canal Flicker, et non les rapports RGB :

- 8 kéch/s actuellement, soit un échantillon toutes les 125 µs ;
- possibilité de tester jusqu'à 14 kéch/s ;
- acquisition continue vers une FIFO de 512 octets ;
- détection d'une variation rapide par rapport à l'éclairage ambiant.

Un flash plus court que 125 µs n'est pas automatiquement raté : son énergie est
intégrée dans un échantillon, ou partagée entre deux échantillons s'il tombe sur
leur frontière. La limite réelle dépendra surtout de l'énergie lumineuse reçue,
du soleil, de la fenêtre optique et du gain choisi.

Les canaux RGB pourraient être exploités plus tard pour mieux reconnaître les
LED rouges annoncées par MSA, mais ils ne sont pas nécessaires pour le premier
essai.

### 4.3 Distance au sol

Le VL53L4CX effectuera une mesure ponctuelle, typiquement toutes les 200 ms. Le
TCS3410 sera arrêté avant l'émission infrarouge 940 nm, puis redémarré après la
mesure.

Cette stratégie simplifie l'électronique et évite les interactions optiques,
mais crée une fenêtre aveugle de plusieurs dizaines de millisecondes pour les
flashs. Le caractère clignotant de l'alarme visuelle est confirmé, mais pas sa
cadence : l'acceptabilité de cette fenêtre devra donc être vérifiée sur la vraie
balise.

## 5. Schéma électrique minimal proposé

~~~text
+3V3 --------------------------------------------------------------+
  |                                                                |
  +-- 10 µF + 100 nF près du connecteur                            |
  |                                                                |
  +-- 22 Ω -- IM68A130A VDD                                        |
  |             +-- 1 µF + 100 nF vers GND                         |
  |                                                                |
  +-- TPS7A2018 1,8 V -- 22 Ω -- TCS3410 VDD                       |
  |          +-- 4,7 µF          +-- 1 µF vers GND                 |
  |                                                                |
  +---------------------------- VL53L4CX AVDD + AVDDVCSEL           |
                                 +-- 100 nF + 4,7 µF vers GND       |
                                                                   |
IM68A130A OUT -- 100 Ω -- MIC_ADC -- PA3 / ADC1_IN4                |
                                                                   |
MicroCAN SDA ------------------- TCS3410 SDA + VL53L4CX SDA         |
MicroCAN SCL ------------------- TCS3410 SCL + VL53L4CX SCL         |
GND ------------------------------------------------ plan continu --+
~~~

Points à conserver dans le schéma du premier prototype :

| Élément | Proposition de départ |
|---|---|
| Entrée 3,3 V | 10 µF + 100 nF près du connecteur |
| Microphone | alimentation via 22 Ω, 1 µF + 100 nF, sortie via 100 Ω |
| Filtre ADC optionnel | empreinte 47 à 100 pF, non montée initialement |
| LDO 1,8 V | TPS7A2018PDBVR, 1 µF en entrée et 4,7 µF en sortie |
| TCS3410 | 22 Ω puis 1 µF au plus près de VDD |
| VL53L4CX | 100 nF + 4,7 µF, XSHUT tiré à 3,3 V par 10 kΩ |
| SDA/SCL | résistances série 0 Ω ; pull-up locales prévues mais non montées |

Le VL53L4CX impose les pointes de courant les plus fortes. Le connecteur, le
faisceau et le rail 3,3 V seront dimensionnés pour au moins 60 mA.

## 6. Connexion à la MicroCAN

La proposition utilise un connecteur verrouillable à six contacts et un petit
faisceau en Y vers J3 et J5 de la MicroCAN.

| Signal carte capteurs | Destination MicroCAN |
|---|---|
| GND_AUDIO | J3 broche 1 |
| MIC_ADC | J3 broche 7 — PA3 / ADC1_IN4 |
| GND_I2C | J5 broche 1 |
| +3V3 | J5 broche 3 |
| SDA | J5 broche 4 — PB7 / I2C1_SDA |
| SCL | J5 broche 5 — PA15 / I2C1_SCL |

Le +5 V de J5 n'est pas utilisé. MIC_ADC devra cheminer avec GND_AUDIO et être
tenu à distance des fils moteur, PWM et alimentation de puissance.

La famille exacte du connecteur reste à choisir avec l'équipe mécanique et en
fonction des connecteurs réellement montés sur la MicroCAN.

## 7. Implantation mécanique proposée

- TCS3410 et VL53L4CX sur la face inférieure, orientés vers le sol.
- Cloison noire entre les deux capteurs optiques.
- Microphone sur l'autre face avec un trou acoustique non métallisé de 0,6 mm
  vers le sol, sous réserve de validation de la variante exacte du microphone.
- Aucun composant, vernis, colle ou sérigraphie devant les surfaces sensibles.
- Keep-out séparés pour les deux zones optiques et le trou acoustique.
- Pour le premier prototype, ouvertures directes sans vitre ni diffuseur afin
  de limiter les inconnues.
- Une carte quatre couches est préférable pour conserver un plan de masse
  continu, mais une carte deux couches reste à discuter si la géométrie est
  simple.

Les dimensions de carte, les fixations, la masse cible et la protection contre
le vent ne sont pas encore définies.

## 8. Disponibilité observée au 22 juillet 2026

Les stocks évoluent rapidement ; ce tableau sert uniquement à éviter de figer
une référence impossible à acheter en petite quantité.

| Référence | Situation observée | Conséquence proposée |
|---|---|---|
| [IM68A130AXTMA1](https://www.digikey.fr/fr/products/detail/infineon-technologies/IM68A130AXTMA1/20115168) | actif, mais stock nul chez Mouser et DigiKey lors de la vérification | vérifier un autre distributeur ou la variante V01 |
| [IM68A130V01XTMA1](https://www.mouser.fr/ProductDetail/Infineon-Technologies/IM68A130V01XTMA1) | actif et préféré, environ 4 100 pièces chez Mouser, vente à l'unité | candidat de remplacement à valider mécaniquement |
| [TCS34103M / Q65114A1293](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/TCS34103M-OLGA6-LF-T-RDP/14123872) | environ 1 600 pièces chez DigiKey, vente à l'unité, Last Time Buy le 31/03/2027 | acceptable ; acheter 10 à 20 pièces rapidement |
| [TSL25213M](https://www.mouser.fr/ProductDetail/ams-OSRAM/TSL25213M) | environ 2 700 pièces chez Mouser | solution de repli proche, non retenue actuellement |
| [VL53L4CXV0DH/1](https://www.mouser.com/ProductDetail/STMicroelectronics/VL53L4CXV0DH-1) | plusieurs milliers de pièces chez Mouser et Farnell | pas de risque immédiat identifié |
| [TPS7A2018PDBVR](https://www.mouser.fr/ProductDetail/Texas-Instruments/TPS7A2018PDBVR) | plus de 10 000 pièces chez Mouser | pas de risque immédiat identifié |

Le choix fonctionnel du microphone reste la famille IM68A130. Avant de libérer
le PCB, il faudra confirmer si la qualification automobile de l'IM68A130A est
utile pour cette compétition ou si l'IM68A130V01, beaucoup plus disponible,
convient. Le symbole, le port acoustique et le land pattern devront être
comparés sur les fiches techniques officielles avant substitution.

## 9. Questions à discuter en équipe

1. Quelles sont la largeur, la cadence, la puissance et la directivité réelles
   des flashs rouges de la balise ?
2. La balise répète-t-elle assez ses flashs pour accepter les fenêtres aveugles
   créées par les mesures ToF ?
3. Quel est le spectre réel de la balise et retrouve-t-on bien trois signaux
   sonores par seconde sous le drone ?
4. L'organisation confirme-t-elle la référence 10088478, sa date de fabrication
   et son bon fonctionnement après l'avis de service MSA de 2026 ?
5. Retient-on l'IM68A130A automobile ou l'IM68A130V01 plus disponible ?
6. Quelle référence de connecteur et quelle longueur de faisceau utiliser ?
7. Quelles dimensions, masse et fixations sont acceptables pour la carte ?
8. Une ouverture directe suffit-elle pour la compétition ou faut-il protéger
   immédiatement les optiques et le microphone contre poussière et vent ?
9. Une carte deux couches est-elle suffisante après placement, ou conserve-t-on
   quatre couches pour réduire le risque sur l'audio et le VCSEL ?
10. Faut-il prévoir une protection ESD montée dès le premier prototype si le
   faisceau reste accessible ?

## 10. Essais proposés avant de figer la carte

- Vérifier les rails 3,3 V et 1,8 V ainsi que les fronts I²C à 400 kHz.
- Enregistrer le microphone, moteurs arrêtés puis tournants, pendant l'autotest,
  la préalarme et l'alarme complète de la vraie balise.
- Filmer ou enregistrer séparément les deux LED d'alarme pendant l'autotest, la
  préalarme et l'alarme complète.
- Vérifier que la LED d'état seule, verte ou rouge à 1 Hz, ne valide jamais une
  détection de flash.
- Tester le TCS3410 à 8 et 14 kéch/s, à l'ombre puis au soleil, avec plusieurs
  largeurs de flash et à environ 1 m.
- Mesurer des sols clairs et sombres entre 0,5 et 1,5 m avec le VL53L4CX.
- Vérifier que le redémarrage du TCS3410 après chaque mesure ToF ne provoque ni
  faux flash ni perte durable de la FIFO.
- Faire les essais avec le câble, la suspension et la mécanique représentatifs
  du montage final.

Ces essais permettront ensuite de figer le gain optique, les seuils logiciels,
la période ToF, la protection mécanique et la nomenclature finale.

## 11. Documents de référence

Documents du projet :

- [étude système](sensors.md) ;
- [schéma MicroCAN v5](../HARDWARE/MICROCAN/MicroCan_v5_schematic.pdf) ;
- [acquisition et détection audio](../COMMON/source/imavRole.cpp) ;
- [gestion lumière et distance](../COMMON/source/imavLightRange.cpp).

Fiches fabricants :

- [Infineon IM68A130A](https://www.infineon.com/assets/row/public/documents/24/49/infineon-im68a130a-datasheet-en.pdf) ;
- [Infineon IM68A130V01](https://www.infineon.com/assets/row/public/documents/24/49/infineon-im68a130-datasheet-en.pdf) ;
- [ams OSRAM TCS3410](https://look.ams-osram.com/m/5b8e2583db0adbb/original/TCS3410-Univers-AL-RGB-Sensor-w-Select-Flicker-Detect-for-Use-Behind-OLED-Displ-or-Aux-to-Cam.pdf) ;
- [ST VL53L4CX](https://www.st.com/resource/en/datasheet/vl53l4cx.pdf) ;
- [TI TPS7A20](https://www.ti.com/lit/ds/symlink/tps7a20.pdf).
