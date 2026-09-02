# CAN IMAV — version café
— Sur le CAN, la MicroCAN envoie trois trucs vraiment utiles : `det`, `snr` et `lit`.
— Les trois partent à 5 Hz en `uavcan.protocol.debug.KeyValue`, sur notre CAN classique à 1 Mbit/s.
— `det`, c'est simple : 0 ou 1, selon que le son de la balise est reconnu ou pas.
— `snr`, c'est la force du bip par rapport au bruit moteur appris autour, pas un niveau sonore absolu.
— Donc pour remonter vers la balise, on compare surtout son évolution pendant le déplacement du drone.
— `lit` utilise la DFT lente pour trouver la balise, puis trois flashs cohérents suffisent pour suivre vite le passage dessus.
— Ce n'est ni une mesure de lux ni un seuil brut : le fond lumineux et le bruit local sont retirés.
— Et on n'impose pas `det ET lit` : un des deux capteurs peut être masqué sans que l'autre raconte n'importe quoi.
— Les trois valeurs arrivent séparément, sans timestamp commun, donc l'autopilote surveille leur fraîcheur.
— Pour l'épreuve, on met `role.imav.debug.publish.optional` à `false` et on ne dépend que de ces trois clés.
— Le gros paquet de messages optionnels, c'est uniquement pour bricoler et comprendre ce qui se passe au banc.
— On y trouve les détails audio : score instantané, fréquence, cadence, énergie et autres valeurs de réglage.
— On a aussi les détails lumière : voie rapide par fronts, score spectral lent, cohérence, couleur et harmonique.
— Les canaux RGBW bruts montent jusqu'à environ 139 Hz pour refaire les FFT tranquillement sur le PC.
— `lct` sert à recoller les groupes et repérer une perte ; `ltu` donne le temps MCU modulo 2^24 microsecondes.
— Si le ToF est activé, `rng` et `rsg` aident au debug, mais la vraie distance passe dans le message UAVCAN standard.
— Avec tout le debug ouvert, on dépasse 1 100 trames par seconde, donc aucune raison de garder ça en vol de concours.
— L'appli Qt et l'enregistreur lisent directement `can0` ; la liaison série n'entre pas dans la chaîne de mesure.
— Bref : en mission on consomme `det`, `snr`, `lit`, et le reste ne sert qu'à nous éviter de régler ça au doigt mouillé.
