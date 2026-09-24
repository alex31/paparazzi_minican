# Définitions des messages UAVCAN custom

Les messages UAVCAN/DroneCAN propres au projet doivent être centralisés dans
`~/DEV/STM32/UAVCAN/DSDL`, sur la branche **`minican`**.

- Avant de créer ou modifier une définition DSDL custom, vérifier que ce dépôt
  partagé est sur la branche `minican` et examiner ses modifications existantes.
- Placer les définitions MicroCAN dans le namespace `microcan` de ce dépôt.
  Ne pas créer de copie indépendante des schémas dans `MICROCAN_V5/DSDL`.
- Générer les codecs dans `~/DEV/STM32/UAVCAN/DSDLC` et utiliser ces codecs
  partagés pour la compilation et les tests du projet.
- Après une modification, vérifier la génération, les identifiants de messages
  et les tests concernés. Conserver dans ce projet la documentation d'utilisation
  et les liens vers les définitions partagées.
