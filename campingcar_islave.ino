	/*
	- mesure des courants des modules a effet hall
	- timer toutes les 20 secondes
	- à disposition du processeur maitre via i2c
	- envoi des donnees de debug liaison serie en bluetooth
	*/

	/*			history
		*	programme de lecture des courants analogiques de batterie de cellule.
		* 	0.0.1 - ecriture initiale menu et mise a l heure
		* 	0.0.2 - affichage des acquisitions
		* 	0.0.3 - affichage courants
		* 	0.0.4 - entree A1 utilisée comme +- correcteur de gain (5 spires)
		* 	0.0.5 - calcul de puissance et moyenne glissante float
		* 	0.0.6 - calcul moyenne glissante = somme -vieux + nouveau / nombre
		* 	0.0.7 - envoi de donnee modele csv
		* 	0.0.8 - envoi de float sur wire
		* 	0.0.9 - correction A/h au lieu de W/h
		* 	0.0.10 - conversion float en long
		* 	0.0.11 - retour au float
		* 	0.0.12 - calcul moyennes en pas CAN
		* 	0.0.13 - commande frigo elect si reception i2c[8]
		* 	0.0.14 - commande frigo affectée sortie 9 pwm timer1
-------------scission programme désulfate en pas can-------------------------------------------------
		* 	0.0.1 - re-ecriture du programme de mesure de courant V0.0.13
		* 	0.0.2 - ajustement des calculs de mesure en mv ! memoire insufisante sdcard
		* 	0.0.3 - stats de mesure en pas can
		* 	0.0.4 - suppression enregistrement sdcard
		* 	0.0.5 - initialisation des moyennes (heure et 24 heures) à la valeur de la 1ere mesure
		* 	achat chargeur de batterie;
		* 	modification de la carte suppression de la commande de puissance du frigo
-------------fusion programme désulfate en pas can-------------------------------------------------
		* 	0.1.0 - integation des modifications
		*	0.1.2 - version print valeur en debug
		*	0.2.0 - mise en eeprom des parametres
		*	0.2.1 - mesure courant d'instrumentation
		*	0.2.2 - initialisation de la moyenne à la 1 ere valeur de courant
		*	0.3.0 - ajout de la variable cumul instantané; cumul(s) en pascan * 60
		*	0.4.0 - mise en sram des variables de cumul
		*	0.4.1 - resolution de cumul * 60
		*	0.4.2 - correction lecture eeprom et setup demarrage minute pleine
		*	0.4.3 - suppression des adherences wire
		*	0.5.0 - softwareserial(tx) utilise pour debug/enregistrement
Md		*	0.5.1 - correction bug setrtc
	*/

// programme :
const char title[] = "Acq_data_I-stat";
const char version[] = "0.5.1";

#include <Arduino.h>
#include <string.h>
#include <I2C_Anything.h>
#include "Timer.h"
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC
#include <Time.h>         //http://www.arduino.cc/playground/Code/Time
#include <Streaming.h>        //http://arduiniana.org/libraries/streaming/
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <SendOnlySoftwareSerial.h>


using namespace std;

/*	recapitulatif des i/o arduino
	#define rx		D0			// tty
	#define tx		D1			// tty
	#define 		D2			//
	#define 		D3			// tx debug/enregistrement
	#define 		D4			//
	#define 		D5			//
	#define 		D6			//
	#define 		D7			//
	#define 		D8			// ledpin
	#define 		D9			//
	#define 		D10			//
	#define mosi	D11			// SPI SD	MOSI
	#define miso	D12			// SPI SD	MISO
	#define sck 	D13			// SPI SD	SCK
	#define 		A0			// V bat
	#define 		A1			// I bat
	#define 		A2			// I instrumentation
	#define 		A3			//
	#define sda	A4			// I2C rtc
	#define scl	A5			// I2C rtc
*/

#define CAPTEUR_COUNT 3

const uint8_t ledPin = 8;			// the number of the LED pin

const unsigned int BAUD_RATE = 19200 ;

// parametres i2c
const uint8_t MY_ADDRESS = 12;
//const int MASTER_ADDRESS = 11;
// nombre d acquisition par minute
uint8_t s_freq = 10 ;				// nombre d acquisition par minute periode 6s = 10

// ces valeurs sont sauvees en EEPROM
const byte EEPROM_ID = 0x89 ;   // used to identify if valid data in EEPROM
//constants used to identify EEPROM addresses
const int ID_ADDR = 0 ;		// the EEPROM address used to store the ID
const int CAPTEUR_ADDR = 1 ;	// adresse en eeprom de la structure capteur
//valeur des variables en eeprom dans l'ordre de la structure capteur
const int default_ANA0[] = { 0, 15000, 0, 1024, 0, 100} ;
const int default_ANA1[] = { -30000, 30000, -512, 512, 0, 100} ;
const int default_ANA2[] = { -11000, 11000, -512, 512, -34, 100} ;
const int default_cumul[] = { 4655, 4655, 4655, 4655} ;

long maxcumul = 279300 ;	// valeur d ecretage du cumul => 100ah => 4654pascan x 60mn; (512 / 11 = 46.54pascan/A)
//long cumuliahg = 0 ;		// cumul energie moyenne glissante ecrete
//long cumuliah = 0 ;		// cumul energie instantannée 60mn ecrete init 6000 = 100ah x 60
//long cumuliaht = 0 ;		// cumul total non ecreté

//	mise en memoire non init des valeurs de cumul pour redemarrage
union configUnion{
	uint8_t    byte[16]; // match the below struct...
	struct {
		long cumuliahg;
		long cumuliah;
		long cumuliaht;
		long chksum;
	} val ;

} config  __attribute__ ( (section (".noinit") ) );

//unsigned long NI_longVar __attribute__( ( section( "NoInit"),zero_init) ) ;

// definition des parametres d entrees analogiques ( 1 voie)
struct Capteur{
	int analog_MIN_elec ;	// valeur elec min
	int analog_MAX_elec ;	// valeur elec max
	int MIN_can ;			// valeur min du can
	int MAX_can ;			// valeur max du can
	int offset ;			// offset pascan conpensation ecart a 0amp
	int gain ;				// * gain de correction ana / 100  (voie courant negative)
};

Capteur* capteurs = (Capteur*) malloc (sizeof(Capteur) * CAPTEUR_COUNT) ;
const int canalI = 2;			// canal courant de batterie pour moyenne glissante
const uint8_t analog_pin[CAPTEUR_COUNT] = {0, 1, 2} ; 					//, 3};
const char* analog_name[CAPTEUR_COUNT] = {"VBat", "Inst", "IBat" } ;	//, "I_ext"};
long analog_value[CAPTEUR_COUNT] = {};

const char* ia_name[2] = {"Am", "Ah" } ; //, "1ia_Min", "1iaH_moy"};

// Variables :
bool debug = false ;   					// mode debug
bool sflag = false ;   					// caractere 's' reçu => mise a l heure
int inm[ CAPTEUR_COUNT ] ;				// registre stat de data minute : moyenne ( cumul / s_freq )
int iam[60] = {0};						// courant moyen / minute pour calculer la moyenne glissante
int iah[24] = {0};						// courant moyen glissant en ampere/heure
long iamsigma = 0 ;						// somme des puissances/minutes
bool init0 = false;

Timer timber;
SendOnlySoftwareSerial SerialDebug (3);  // Tx pin

// various commands we might get

union mesure
	{
		byte bval[4] ;
		unsigned int ival[2] ;
		unsigned long lval ;
		float fval ;
	} valeur ;

//			****	sous programmes		****

void(* resetFunc) (void) = 0; //declare reset function @ address 0

int freeRam () {
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
} // end of freeRam

void ana_get(){			// acquisition analogiques
	for(int i=0;i<=CAPTEUR_COUNT-1;i++){
		long val = analogReadPin(analog_pin[i]);
		if(debug){
			Serial << val << F(" | ");
		}

		// valeur mesuree en pascan X par le gain
		val = (val * capteurs[i].gain) / 100 ;
		if(debug){
			Serial << val << F(" | ");
		}

		// recallage du CAN arduino avec echelle en eeprom
		val = map( val , 0, 1024, (capteurs[i].MIN_can), (capteurs[i].MAX_can));
		val += capteurs[i].offset;
		analog_value[i] +=  val;
		if(debug){
			Serial << val << F(" | ") << map(val , capteurs[i].MIN_can, capteurs[i].MAX_can, capteurs[i].analog_MIN_elec, capteurs[i].analog_MAX_elec) << endl;
		}

	}

} // end of ana_get

/** Mesure entree ANA avec la référence interne à 1.1 volts */
unsigned int analogReadPin(byte pin) {

	/** Sélectionne l entee ADC avec la référence interne à 1.1 volts */
	//ADMUX = bit (REFS0) | bit (REFS1)  | pin;    // input pin
	//ADMUX =   bit (REFS0) | (pin & 0x07);  // AVcc
	/** Sélectionne l entee ADC  */
	ADMUX = bit (REFS0) | pin ;    // input pin
	delay (200);  // let it stabilize

	/** clear prescaler bits**/
	ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2));
	/** Activation du prescaler**/
	//ADCSRA |= bit (ADPS0) | bit (ADPS2);                 //  32
	ADCSRA |= bit (ADPS1) | bit (ADPS2);                 //  64
	/** Active le convertisseur analogique -> numérique **/
	ADCSRA |= (1 << ADEN);
	/** Lance une conversion analogique -> numérique **/
	ADCSRA |= (1 << ADSC);
	/** Attend la fin de la conversion **/
	while(ADCSRA & (1 << ADSC));
	/** Récupère le résultat de la conversion **/
	return ADCL | (ADCH << 8);
} // end of analogReadPin

void set_rtcTime(){		//set rtc time
	//check for input to set the RTC, minimum length is 12, i.e. yy,m,d,h,m,s
	//note that the tmElements_t Year member is an offset from 1970,
	//but the RTC wants the last two digits of the calendar year.
	//use the convenience macros from Time.h to do the conversions.
	time_t t;
	tmElements_t tm;
	int y = Serial.parseInt();
	if (y >= 100 && y < 1000){
		Serial << F("Error: Year must be two digits or four digits!") << endl;
	}
	else {
		if (y >= 1000){
			tm.Year = CalendarYrToTm(y);
		}
		else {		//(y < 100)
			tm.Year = y2kYearToTm(y);
		}
		tm.Month = Serial.parseInt();
		tm.Day = Serial.parseInt();
		tm.Hour = Serial.parseInt();
		tm.Minute = Serial.parseInt();
		tm.Second = Serial.parseInt();
		t = makeTime(tm);
		RTC.set(t);        //use the time_t value to ensure correct weekday is set
		setTime(t);
		char buffer[20] ;
		char* bfr = buffer ;
		digitalClockDisplay (bfr) ;
		Serial.print(buffer) ;
		Serial << endl;
		//dump any extraneous input
		while (Serial.available() > 0) Serial.read();
	}
} // end of set_rtcTime

byte get_list(char *liste){		//get list of variables

	for(int i= 0;i<=CAPTEUR_COUNT-1;i++){
		strcat(liste, analog_name[i]);
		strcat(liste, ",");
	}
	for(int i= 0;i<=1;i++){
		strcat(liste, ia_name[i]);
		strcat(liste, ",");
	}
	strcat(liste, "cumuliahg");
	strcat(liste, ",");
	strcat(liste, "cumuliah");
	strcat(liste, ",");
	strcat(liste, "cumuliaht");
	strcat(liste, "\n");
	return strlen(liste);
} // end of get_list

char* digitalClockDisplay(char* buffer){
	int jour = day();
	int mois = month();
	int annee = year();
	int heure = hour();
	int minutes = minute();
	int secondes = second();
	sprintf(buffer,"%04d/%02d/%02d %02d:%02d:%02d", annee, mois, jour, heure ,minutes ,secondes);  //  %d pour un int
	//sprintf(buffer,"%02d:%02d:%02d %02d/%02d/%04d", heure ,minutes ,secondes, jour, mois, annee);  //  %d pour un int
	return buffer ;
} // end of digitalClockDisplay


void print_title (){
	Serial.println();
	Serial.print(title);
	Serial.print(F(" "));
	Serial.println(version);
} // end of print_title


void print2serial(char *bfr, int iam, int iah){

	//Serial.print(F("freeRam : "));
	//Serial.println(freeRam());

	// impression de la date et heure
	Serial.print(bfr) ;
	Serial.print(F(",")) ;
	SerialDebug.print(bfr) ;
	SerialDebug.print(F(",")) ;
	// impression mesures des capteurs

	for(int i=0;i<=CAPTEUR_COUNT-1;i++){
		Serial << map(inm[i], capteurs[i].MIN_can, capteurs[i].MAX_can, capteurs[i].analog_MIN_elec, capteurs[i].analog_MAX_elec) << F(",");
		SerialDebug << map(inm[i], capteurs[i].MIN_can, capteurs[i].MAX_can, capteurs[i].analog_MIN_elec, capteurs[i].analog_MAX_elec) << F(",");
	}
	/** impression sommes des acquisitions minutes glissantes
	Serial.print(map(iamsigma , 0, 61380, analog_IMIN[1], analog_IMAX[1]) );
	Serial.print(F(",")) ;
	// impression courant instantané minute -- impression initile => copie inm[1]
	Serial.print( map(iam , -512, 511, capteurs[i].MIN_can, capteurs[i].MAX_can) );
	Serial.print(F(",")) ;**/
	// impression moyenne glissante du courant sur 60 minutes
	Serial << map(iah , capteurs[canalI].MIN_can, capteurs[canalI].MAX_can, capteurs[canalI].analog_MIN_elec, capteurs[canalI].analog_MAX_elec) << F(",");
	SerialDebug << map(iah , capteurs[canalI].MIN_can, capteurs[canalI].MAX_can, capteurs[canalI].analog_MIN_elec, capteurs[canalI].analog_MAX_elec) << F(",");
	//impression du cumul de capacité
	Serial << map((config.val.cumuliahg / 60) , capteurs[canalI].MIN_can, capteurs[canalI].MAX_can, capteurs[canalI].analog_MIN_elec, capteurs[canalI].analog_MAX_elec) << F(",");
	SerialDebug << map((config.val.cumuliahg / 60) , capteurs[canalI].MIN_can, capteurs[canalI].MAX_can, capteurs[canalI].analog_MIN_elec, capteurs[canalI].analog_MAX_elec) << F(",");
	Serial << map((config.val.cumuliah / 60) , capteurs[canalI].MIN_can, capteurs[canalI].MAX_can, capteurs[canalI].analog_MIN_elec, capteurs[canalI].analog_MAX_elec) << F(",");
	SerialDebug << map((config.val.cumuliah / 60) , capteurs[canalI].MIN_can, capteurs[canalI].MAX_can, capteurs[canalI].analog_MIN_elec, capteurs[canalI].analog_MAX_elec) << F(",");
	Serial << map((config.val.cumuliaht / 60) , capteurs[canalI].MIN_can, capteurs[canalI].MAX_can, capteurs[canalI].analog_MIN_elec, capteurs[canalI].analog_MAX_elec) << F("\n");
	SerialDebug << map((config.val.cumuliaht / 60) , capteurs[canalI].MIN_can, capteurs[canalI].MAX_can, capteurs[canalI].analog_MIN_elec, capteurs[canalI].analog_MAX_elec) << F("\n");
	//Serial.print(F("freeRam : "));
	//Serial.println(freeRam());
}// end of print2serial

/**
 * 	int analog_MIN_elec ;	// valeur elec min
	int analog_MAX_elec ;	// valeur elec max
	int MIN_can ;			// valeur min du can
	int MAX_can ;			// valeur max du can
	int offset ;			// offset pascan conpensation ecart a 0amp
	int gain ;				// * gain de correction ana / 100  (voie courant negative)
**/

void print_code_integer(int index){
	if (index ==0){Serial.print(F("ID EEPROM = ")) ;}
	else if (index ==1){Serial.print("analog_MIN_elec_0 = ") ;}
	else if (index == 3){Serial.print(F("analog_MAX_elec_0 = ")) ;}
	else if (index == 5){Serial.print(F("MIN_can_0 = ")) ;}
	else if (index == 7){Serial.print(F("MAX_can_0 = ")) ;}
	else if (index == 9){Serial.print(F("offset_0 = ")) ;}
	else if (index == 11){Serial.print(F("gain_0 = ")) ;}
	else if (index == 13){Serial.print(F("analog_MIN_elec_1 = ")) ;}
	else if (index == 15){Serial.print(F("analog_MAX_elec_1 = ")) ;}
	else if (index == 17){Serial.print(F("MIN_can_1 = ")) ;}
	else if (index == 19){Serial.print(F("MAX_can_1 = ")) ;}
	else if (index == 21){Serial.print(F("offset_1 = ")) ;}
	else if (index == 23){Serial.print(F("gain_1 = ")) ;}
	else if (index == 25){Serial.print(F("analog_MIN_elec_2 = ")) ;}
	else if (index == 27){Serial.print(F("analog_MAX_elec_2 = ")) ;}
	else if (index == 29){Serial.print(F("MIN_can_2 = ")) ;}
	else if (index == 31){Serial.print(F("MAX_can_2 = ")) ;}
	else if (index == 33){Serial.print(F("offset_2 = ")) ;}
	else if (index == 35){Serial.print(F("gain_2 = ")) ;}
}	// End of print_code_integer

void printEEPROM(){
	int integer = 0;
	byte octet = 0;
	EEPROM_readAnything(ID_ADDR, octet);
	Serial << F("EEPROM_ID attendu = ") << EEPROM_ID << F(" EEPROM_ID lu = ") << octet << endl;
	if(octet == EEPROM_ID){
		Serial << F("EEPROM_ID correct") << endl;
	}
	else{
		Serial << F("erreur EEPROM_ID ") << octet << endl;
	}
	for (int i = 1; i <= 35; i+=2){
		Serial.print(i);
		print_code_integer(i) ;
		EEPROM_readAnything(i, integer);
		Serial.println(integer) ;
	}
	for (int i = 37; i <= 49; i+=4){
		Serial.print(i);
		print_code_integer(i) ;
		EEPROM_readAnything(i, integer);
		Serial.println(integer) ;
	}
}	// End of printEEPROM

void printCapteurs(){
	for (int k=0; k<=CAPTEUR_COUNT -1; k++){
		Serial << F("analog_MIN_elec_0 = ") << capteurs[k].analog_MIN_elec << endl;
		Serial << F("analog_MAX_elec_0 = ") << capteurs[k].analog_MAX_elec << endl;
		Serial << F("MIN_can_0 = ") << capteurs[k].MIN_can << endl;
		Serial << F("MAX_can_0 = ") << capteurs[k].MAX_can << endl;
		Serial << F("offset_0 = ") << capteurs[k].offset << endl;
		Serial << F("gain_0 = ") << capteurs[k].gain << endl;
	}

}	// End of printCapteurs

// Write any data structure or variable to EEPROM
int EEPROMAnythingWrite(int pos, char *zeichen, int lenge){
	for (int i = 0; i < lenge; i++){
		EEPROM.write(pos + i, *zeichen);
		zeichen++;
	}
	return pos + lenge;
}

// Read any data structure or variable from EEPROM
int EEPROMAnythingRead(int pos, char *zeichen, int lenge){
	for (int i = 0; i < lenge; i++){
		*zeichen = EEPROM.read(pos + i);
		zeichen++;
	}
	return pos + lenge;
}

uint16_t getchksum() {
	long sum = 0;
	for (int position = 0; position < (sizeof(config) - sizeof(config.val.chksum)); position++) {
		sum = sum + config.byte[position];
	}
	return sum;
}

//			****	fin des sous programmes		****

void setup(){

	pinMode(ledPin, OUTPUT);
	digitalWrite(ledPin, HIGH);		// turn the LED on (HIGH is the voltage level)
	Serial.begin(BAUD_RATE);		// initialize serial:
	SerialDebug.begin(BAUD_RATE);	// initialize serialdebug:
	print_title () ;
	setSyncProvider(RTC.get);
	if (timeStatus() != timeSet){
		Serial.print(F(" FAIL remote RTC!\n"));
	}
	int rc = 0 ;
	Capteur capteur;
	long sum = getchksum();
	Serial << F("sum : ") << sum << F("; checksum : ") << config.val.chksum << endl;
	if(sum != config.val.chksum or sum == 0){	//	c'est un demarrage à froid
		Serial << F("demarrage a froid ") << endl;
		long val = default_cumul[1] ;
		config.val.cumuliahg = val * 60;
		val = default_cumul[2];
		config.val.cumuliah = val * 60;
		val = default_cumul[2];
		config.val.cumuliaht = val * 60;
		config.val.chksum = getchksum();
	}
	// valeurs de cumul au demarrage
	Serial << F("param maxcumul : ") << maxcumul << endl;
	Serial << F("param cumuliahg : ") << config.val.cumuliahg << endl;
	Serial << F("param cumuliah : ") << config.val.cumuliah << endl;
	Serial << F("param cumuliaht : ") << config.val.cumuliaht << endl;
	Serial << F("param chksum : ") << config.val.chksum << endl;

	//EEPROM_readAnything(ID_ADDR, id);
	byte id = EEPROM.read(ID_ADDR);
	Serial << F("EEPROM_ID attendu = ") << EEPROM_ID << F(" EEPROM_ID lu = ") << id << endl;
	if(id == 255){
		id = EEPROM.read(0);
		Serial << F("EEPROM_ID attendu = ") << EEPROM_ID << F(" EEPROM_ID lu = ") << id << endl;
	}
	if(id == EEPROM_ID){
		Serial << F("EEPROM_ID correct") << endl;
		int addr = CAPTEUR_ADDR;
		for (int k=0; k<=CAPTEUR_COUNT -1; k++){
			// Read structure (struct) from EEPROM
			//int nekst_free = EEPROMAnythingRead(addr, reinterpret_cast<char*>(&capteurs[k]), sizeof(capteur));
			rc = EEPROM_readAnything(addr, capteur);
			Serial << F("analog_MIN_elec_0 = ") << capteur.analog_MIN_elec << endl;
			Serial << F("analog_MAX_elec_0 = ") << capteur.analog_MAX_elec << endl;
			Serial << F("MIN_can_0 = ") << capteur.MIN_can << endl;
			Serial << F("MAX_can_0 = ") << capteur.MAX_can << endl;
			Serial << F("offset_0 = ") << capteur.offset << endl;
			Serial << F("gain_0 = ") << capteur.gain << endl;
			Serial << F("Lecture EEPROM = ") << rc << F(" Octets") << endl;
			memcpy ( &capteurs[k], &capteur, sizeof(capteur) );
			addr = addr + sizeof(capteur);
		}
	}
	else {
		Serial << F("erreur lecture EEPROM_ID = ") << id << endl;
		int rc = EEPROM_writeAnything(ID_ADDR, EEPROM_ID);
		int addr = CAPTEUR_ADDR;
		rc = EEPROM_writeAnything(addr, default_ANA0);
		Serial << F("copie EEPROM valeurs default_ANA0 ") << rc << " octets"<< endl;
		addr = addr + sizeof(default_ANA0);
		rc = EEPROM_writeAnything(addr, default_ANA1);
		Serial << F("copie EEPROM valeurs default_ANA1 ") << rc << " octets"<< endl;
		addr = addr + sizeof(default_ANA1);
		rc = EEPROM_writeAnything(addr, default_ANA2);
		Serial << F("copie EEPROM valeurs default_ANA2 ") << rc << " octets"<< endl;
		addr = addr + sizeof(default_ANA2);

		memcpy ( &capteurs[0].analog_MIN_elec, &default_ANA0, sizeof(default_ANA0) );
		memcpy ( &capteurs[1].analog_MIN_elec, &default_ANA1, sizeof(default_ANA1) );
		memcpy ( &capteurs[2].analog_MIN_elec, &default_ANA2, sizeof(default_ANA2) );
		if(debug){
			printEEPROM();
			printCapteurs();
		}
	}

	//initialisation arrays
	Serial.print(F("attente 1 minute d acquisitions\n"));
	time_t t;
	do{
		for(int i=0;i<=CAPTEUR_COUNT-1;i++){
			analog_value[i] =  0;
		}
		ana_get() ;	// uncoup pour rire
		iamsigma = 0;
		for(int j=0;j<=59;j++){			// raz de la moyenne glissante de puissance
			iam [j] = analog_value[canalI] ;
			iamsigma += analog_value[canalI] ;
		}
		for(int i=0;i<=24-1;i++){
			iah[i] = analog_value[canalI];
		}
		Serial << F(".");
		t = now();
	}while (second(t) !=0);
	//Serial << F("analog_value[canalI] :") << analog_value[canalI] << endl;
	Serial << endl;
	init0 = true;
	timber.oscillate(ledPin, 100, LOW, 10);
	Serial<< F("Date , Tension, Inst, Courant, ia_MoyGlHeure, ia_Cumulgl, ia_Cumul, ia_cumult\n") << endl;

} // end of setup

void loop(void){
	static time_t tLast;
	static int count ;
	time_t t;
	t = now();
	if (sflag == false){
		if (t != tLast) {
			tLast = t;
			if (second(t) % (60/s_freq) == 0 ) {		//acquisition des mesures
				timber.pulse(ledPin, 500, LOW);
				ana_get() ;
				count ++;
			}
			if (second(t) == 0) {						// calculs des mesures moyenne / minute

				for(int i=0;i<=CAPTEUR_COUNT-1;i++){
					inm[i] = analog_value[i] / s_freq;
					analog_value[i] = 0 ;
					if(debug){
						Serial << F("inm[i] : ") << inm[i] << F(" / ") ;
					}
				}
				Serial << endl;
				// tous les calculs restent en pascan
				int minut = minute(t);
				// capacité totale 60 minutes - ( ia vielle minute )
				iamsigma = iamsigma - iam [minut] ;
				// mise a jour capacité minute actuelle
				iam[minut] = inm[canalI] ;		// Ampère / minute (pascan)
				// capacité totale 59 minutes + ( ia nouvelle minute )
				iamsigma = iamsigma + iam [minut] ;
				int heure = hour();
				//calcul capacité moyenne gissante horaire
				iah[heure] = iamsigma  / 60 ;
				//calcul capacité cumulée ah/mn (pascan)
				long sum = getchksum();
				if (sum != config.val.chksum){
					Serial << F("erreur config.val.chksum : ")<< config.val.chksum << F("sum : ") << sum << endl;
				}
				config.val.cumuliahg += iah[heure];	// ajout de la capacité minute moyenne glissante (pascan)
				config.val.cumuliah += iam[minut];		// ajout de la capacité minute (pascan)
				config.val.cumuliaht += iam[minut];	// ajout de la capacité minute (pascan)
				if(config.val.cumuliahg > maxcumul){
					config.val.cumuliahg = maxcumul;
				}
				if(config.val.cumuliah > maxcumul){
					config.val.cumuliah = maxcumul;
				}
				config.val.chksum = getchksum();
				if(debug){
					Serial << F("param cumuliahg : ") << config.val.cumuliahg << endl;
					Serial << F("param cumuliah : ") << config.val.cumuliah << endl;
					Serial << F("param cumuliaht : ") << config.val.cumuliaht << endl;
					Serial << F("param chksum : ") << config.val.chksum << endl;
					Serial << F("iamsigma : ") << iamsigma << F(" | iah_gl : ")
							<< iah[heure] << F(" | cumuliahg : ") << config.val.chksum << endl;
					Serial.println() ;
					for(int i=0;i<=60-1;i++){
						Serial << F("|") << iam[i];
					}
					Serial << endl ;
					for(int i=0;i<=24-1;i++){
						Serial << F("|") << iah[i];
					}
					Serial << endl ;
				}
				char buffer[20] ;
				char *bfr = buffer ;
				digitalClockDisplay (bfr) ;
				print2serial(bfr, iam[minut], iah[heure]);
				//Serial.print(F("freeRam : "));
				//Serial.println(freeRam());
			}
		}

		timber.update();	// mise à jour du timer*/
	}
} // end of loop

void serialEvent() {
	if (!sflag){
		if (Serial.available()) {
			timber.pulse(ledPin, 100, LOW);
			char octet = Serial.read();
			if (octet == 'h' or octet == '?'){
				Serial.print(F("tapper : \n"));
				Serial.print(F("h ou ? -> ce menu\n"));
				Serial.print(F("c -> reset - cumuls\n"));
				Serial.print(F("d -> debug - reglage\n"));
				Serial.print(F("f -> frequence - fx\n"));
				Serial.print(F("l -> liste des variables\n"));
				Serial.print(F("g -> get/set val eeprom\r\n"));
				Serial.print(F("p -> stat puissance 24H\n"));
				Serial.print(F("m -> stat puissance Horaire\n"));
				Serial.print(F("r -> reset avr\n"));
				Serial.print(F("syy,m,d,h,m,s -> mise a l heure\n"));
				Serial.print(F("v -> version\n"));
			}
			else if(octet == 's'){
				sflag = true;
				Serial << F("tapper la date au format yy,mm,dd,hh,mm,ss") << endl;
				if (Serial.available() >= 12){
					set_rtcTime();
				}
			}
			else if(octet == 'd'){
				debug = !debug ;
				Serial.print(F("debug = "));
				Serial.println( debug);
			}
			else if(octet == 'l'){
				char buffer[63] ;
				char* bfr = buffer ;
				get_list(bfr);
				Serial.println(buffer) ;
			}
			else if(octet == 'c'){
				// reset cumuls
				long val = default_cumul[1] ;
				config.val.cumuliahg = val * 60;
				val = default_cumul[2];
				config.val.cumuliah = val * 60;
				val = default_cumul[2];
				config.val.cumuliaht = val * 60;
				config.val.chksum = getchksum();
				resetFunc() ;  //call reset
			}
			else if(octet == 'f'){
				delay(5);
				if (Serial.available() > 0){
					uint8_t val = Serial.parseInt();
					Serial.print(F("s_freq : "));
					Serial.print(s_freq);
					Serial.print(F("|"));
					s_freq = val;
					Serial.println(s_freq);
				}
			}
			else if (octet == 'g'){
				int index ;
				byte octet ;
				int integer ;
				delay(5);
				if (Serial.available() > 0){		// index de donnee à lire present
					index = Serial.parseInt();
					if (index == 0){
						EEPROM_readAnything(index, octet);
						Serial.println(octet, HEX) ;
					}
					else {							// lecture eeprom
						EEPROM_readAnything(index, integer);
						print_code_integer(index) ;
						Serial.println(integer) ;
					}
					delay(  5);
					if (Serial.available() > 0){		// valeur de donnee à ecrire presente
						Serial.print(" : ");
						int val = Serial.parseInt();
						int rc = EEPROM_writeAnything(index, val);
						EEPROM_readAnything(index, integer);
						if (integer == val){
							Serial << F("Enregistrement verifie :) ") << endl;
						}
						else{
							Serial << F("Erreur à l'enregistrement :( !!!!") << endl;
						}
					}
				}
				else {
					printEEPROM();
					printCapteurs();
				}
				Serial << F(" tapper P pour activer Print_data ") << endl ;
			}
			else if (octet == 'r'){
				resetFunc() ;  //call reset
			}
			else if (octet == 'p'){
				for(int i=0;i<=24-1;i++){
					Serial << F("|") << iah[i];
				}
				Serial << endl ;
			}
			else if (octet == 'm'){
				for(int i=0;i<=60-1;i++){
					Serial << F("|") << iam[i];
				}
				Serial << endl ;
			}
			else if (octet == 'v'){
				print_title() ;
			}
			else {
				Serial << F("commande inconnue : ") << octet << F(" tapper h pour l aide. ") << endl;
			}
		}
	}
	else if (sflag){
		if (Serial.available() >= 12) {
			set_rtcTime();
		}
	}
} // end of serialEvent


