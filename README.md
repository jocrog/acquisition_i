# acquisition_i
mesure et statistique du bilan journalier de charge/decharge de la batterie de la cellule d'un camping-car en autonomie

capacité de batterie : 100 Ah -> decharge max 50 % = 50 Ah
décharge journalière été = 25 Ah
decharge journaliere hiver 35 Ah ( electronique et ventilation chauffage )
charge alternateur 10 Ah ( -> 2H30 / jour de moteur si absence de soleil )
charge solaire ( 2 panneaux de 100 W ) valeur max mesurée à 10 Ah le 21 juin ensoleillé au centre de la france

mesure du courant de charge et decharge par un capteur a effet hall ( LEM LA55-P ) couplé a un montage a amplis operationnels
echelle capteur -50mA 50mA; l'echelle avr328 de mesure est de 0v 5v pour -11A +11A

les calculs et statistiques sont effectués par un AVR328P( arduino ) en config mini 1 quartz, 2 capa 18pf synchronisé par un module DS3231
moyenne de 10 acquisitions de courant/ minute, moyenne glissante en Ah, cumul des moyennes( instantannee, glissante, totale ).

edition du programme linux/geany -> ino clean, ino build :  https://influence-pc.fr/10-07-2012-alternative-a-lide-arduino-decouvrez-ino-geany
mise a jour du programme via liaison iscp ( spi) avrdude et makefile -> commande : sudo make upload

les donnees sont envoyees toutes les minutes a l utilisateur par la liaison serie au format csv :
"date		hour",		"VBat",		"Inst",		"IBat",		"Ah",		"cumuliahg",	"cumuliah",		"cumuliaht"
et aussi vers une deuxieme liaison serie pour enregistrement sur une carte sd geree par un autre avr328.

un menu permet de parametrer les variables modifiables en eeprom et la mise a l heure rtc
