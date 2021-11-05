# acquisition_i
mesure et statistique du bilan journalier de la batterie de cellule d'un camping-car en autonomie

capacité de baterie : 100 Ah -> decharge max 50 % = 50 Ah
décharge journalière été = 25 Ah
decharge journaliere hiver 35 Ah ( electronique et ventilation chauffage )
charge alternateur 10 Ah ( -> 2 H de moteur si absence de soleil )
charge solaire ( 2 panneaux de 100 W ) mesurée à 10 Ah le 21 juin ensoleillé au centre de la france

mesure du courant de charge et decharge par un capteur a effet hall ( LEM LA55-P ) couplé a un montage a amplis operationnels
l'echelle de mesure est de 0 - 5v pour -11A - 11A

les calculs et statistiques sont effectués par un AVR328P ( arduino ) synchronisé par un module DS3231

les donnees sont envoyees a l utilisateur par la liaison serie via un module bluetooth au format csv
un menu permet de parametrer les variables modifiables en eeprom et la mise a l heure rtc
