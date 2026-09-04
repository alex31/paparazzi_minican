# Émulateur de balise MSA motionSCOUT K-T-R

Ce profil est destiné aux essais IMAV avec une carte H7, une sortie
haut-parleur et une rangée de LED RGB. Il reproduit les caractéristiques
connues de la balise sans prétendre remplacer une mesure sur l'exemplaire de
la compétition.

## Cible

- modèle : **MSA motionSCOUT K-T-R** ;
- référence : **10088478** ;
- alarme sonore annoncée par le règlement IMAV : **2,6 à 3,0 kHz** et
  **95 dB à 3 m** ;
- préalarme officielle : **2 signaux par seconde**, avec niveau sonore
  croissant pendant environ 15 s ;
- alarme complète officielle : **3 signaux par seconde**, au niveau maximal ;
- signal optique : deux LED rouges d'alarme très lumineuses.

Le manuel indique que la préalarme commence après environ 25 s sans mouvement
et que l'alarme complète commence environ 15 s plus tard. L'alarme complète
peut aussi être déclenchée immédiatement par le bouton d'alarme.

## Profil de banc retenu

Le fichier `motionscout_ktr_full_alarm_3hz.wav` représente directement l'état
utile pendant l'épreuve :

- WAV PCM signé 16 bits, mono, 48 kHz ;
- durée de 10 s, bouclable ;
- un chirp linéaire de 2,1 à 2,5 kHz pendant 120 ms ;
- répétition toutes les 333,333 ms, soit 3 Hz ;
- crête numérique à -6 dBFS.

La plage de 2,1 à 2,5 kHz et la forme en chirp viennent d'une analyse de vidéo
de démonstration, dont le maximum spectral se situe vers 2 433 Hz. Il s'agit
d'une observation, pas d'une spécification MSA. Elle est retenue parce qu'elle
exerce mieux la bande complète du détecteur embarqué que ne le ferait une
sinusoïde fixe.

Le fichier `motionscout_ktr_test_sequence.wav` enchaîne une seconde de silence,
15 s de préalarme à 2 Hz avec quatre paliers de niveau, une seconde de silence,
puis 10 s d'alarme complète à 3 Hz. Il sert à vérifier le changement de cadence
et la montée de niveau ; pour imiter la balise du mannequin, utiliser plutôt la
boucle d'alarme complète.

Le niveau numérique ne fixe pas le niveau acoustique. Commencer à faible
volume : **95 dB à 3 m est un niveau dangereux pour un essai de proximité**.
La calibration en dB SPL dépend du haut-parleur, de l'amplificateur, de
l'alimentation, de la directivité et de la distance.

## Pattern des LED

Pour l'alarme complète à reproduire sur la rangée RGB :

1. toutes les LED, ou deux groupes représentant les deux LED de la balise,
   passent simultanément à `RGB(255, 0, 0)` ;
2. elles restent allumées 100 ms ;
3. elles restent éteintes 233,333 ms ;
4. le cycle recommence, soit 3 flashs par seconde.

Le front montant du flash est aligné sur le début du chirp. Le son dure 120 ms,
donc 20 ms de plus que le flash. Pour une préalarme optionnelle, employer le
même flash de 100 ms toutes les 500 ms, soit 2 Hz.

Les 100 ms, l'allumage simultané et l'alignement avec le son sont des choix
d'émulation fondés sur des vidéos. Le fabricant ne publie ni durée, ni cadence,
ni synchronisation garanties pour les LED d'alarme. Le fichier
`motionscout_ktr_pattern.csv` sépare donc les valeurs officielles des choix de
banc.

Ne pas ajouter au pattern principal le petit voyant d'état bicolore de la
balise : son clignotement à 1 Hz est distinct des deux LED rouges d'alarme.

## Chronogramme H7 minimal

```text
alarme complète, période = 333333 us

temps dans le cycle  LED rouge   audio
0 ... 100 ms         ON          chirp 2,1 -> 2,5 kHz
100 ... 120 ms       OFF         fin du chirp
120 ... 333,333 ms   OFF         silence
```

À chaque échantillon audio `n` du chirp, une synthèse directe peut utiliser une
phase continue :

```text
t = n / 48000
f(t) = 2100 + (2500 - 2100) * t / 0.120
phase += 2*pi*f(t)/48000
sample = gain * envelope(t) * sin(phase)
```

Une rampe de 5 ms au début et à la fin du chirp évite les clics. Pour une boucle
de test identique au WAV, utiliser un gain de 0,501 (-6 dBFS).

## Validation sur le banc PC

La MicroCAN a été testée avec une enceinte reliée à la carte son du PC :

- chirps de 60, 80, 100 et 120 ms à 3 Hz : détection valide ;
- séquence complète : 1,98–2,04 Hz en préalarme et 2,96–3,04 Hz en alarme ;
- chirps seuls jusqu'à -30 dBFS : détection valide sur ce banc ;
- tonalité à 1 kHz, bruit blanc continu et bruit blanc pulsé à 3 Hz : rejetés ;
- bruit blanc continu à -30 dBFS appris pendant 3 s, puis chirp à -24 dBFS :
  détection stable à 3 Hz.
- lors de la validation initiale à publication CAN permanente, trois niveaux
  espacés de 6 dBFS ont produit environ 12,0, 17,6 et 23,7 dB de SNR relatif.
  La publication `snr` est désormais déclenchée à la fin de chaque bip reconnu
  et doit être revalidée avec `role.imav.audio.snr_alpha=1`.

Les dBFS décrivent seulement les fichiers numériques. Ils ne donnent ni le
SNR acoustique au microphone, ni un niveau SPL transposable au drone.

## Sources locales

- [Règlement IMAV 2026 v2.1](Rulebook_IMAV2026_V2.1.pdf), section 5.4.7 ;
- [manuel MSA motionSCOUT 10251777/01](OPM_motionSCOUT_10251777_01_Book.pdf) ;
- [étude capteurs et observations vidéo](sensors_electronic.md), section 1.2.
