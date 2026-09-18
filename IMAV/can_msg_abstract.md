# CAN IMAV — version café
— Sur le CAN, la MicroCAN envoie deux scores utiles : `snr` pour le son et `lit` pour la lumière.
— `snr` part une fois par bip sonore reconnu et `lit` une fois par flash lumineux reconnu, donc chacun vers 3 Hz en régime établi.
— `snr`, c'est la force du bip par rapport au bruit moteur appris autour, pas un niveau sonore absolu.
— Son IIR est réglé par `role.imav.audio.snr_alpha` : 1 donne le pic brut sans lissage, 0,5 conserve l'ancien comportement moitié-moitié.
— Donc pour remonter vers la balise, on compare surtout son évolution pendant le déplacement du drone.
— En mode standard, `lit` vient d'une fenêtre glissante d'une seconde et part sur le front descendant estimé.
— L'option `role.imav.light.adaptive_pattern` conserve la détection standard et ajoute les trois flashs + pause et le rythme lent de la vidéo. Ces nouveaux motifs sont appris après trois cycles et publient sur les flashs observés ; CREE reste séparé.
— Ce n'est ni une mesure de lux ni un seuil brut : le fond lumineux et le bruit local sont retirés.
— `snr` guide la descente de gradient à longue portée ; `lit` sert à confirmer la proximité et déclencher le largage du medikit.
— Après 1,5 s sans salve, `snr` passe à zéro ; `lit` passe à zéro après perte du motif, avec un délai de secours de deux périodes en standard ou 1,8 s en adaptatif.
— Ces zéros sont ensuite répétés à 1 Hz : la perte d'une trame ne laisse donc pas un ancien score actif.
— Pour l'épreuve, on met `role.imav.debug.publish.optional` à `false` et on ne dépend que de ces deux clés.
— Le gros paquet de messages optionnels, c'est uniquement pour bricoler et comprendre ce qui se passe au banc.
— On y trouve les détails audio : score instantané, fréquence, cadence, énergie et autres valeurs de réglage.
— Côté lumière, on voit aussi les temps haut/bas, le motif choisi et si ses harmoniques ont vraiment la bonne forme.
— Les canaux RGBW bruts montent jusqu'à environ 139 Hz pour refaire les FFT tranquillement sur le PC.
— `lct` sert à recoller les groupes et repérer une perte ; `ltu` donne le temps MCU modulo 2^24 microsecondes.
— Si le ToF est activé, `rng` et `rsg` aident au debug, mais la vraie distance passe dans le message UAVCAN standard.
— Avec tout le debug ouvert, on approche 1 100 trames par seconde, donc aucune raison de garder ça en vol de concours.
— L'appli Qt et l'enregistreur lisent directement `can0` ; la liaison série n'entre pas dans la chaîne de mesure.
— Bref : en mission on consomme `snr` et `lit`, et le reste ne sert qu'à nous éviter de régler ça au doigt mouillé.
