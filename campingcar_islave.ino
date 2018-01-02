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
M		* 	0.0.14 - commande frigo affectée sortie 9 pwm timer1
-------------fusion programme désulfate en pas can-------------------------------------------------
*		* 	0.0.1 - re-ecriture du programme de mesure de courant V0.0.13
		* 	0.0.2 - ajustement des calculs de mesure en mv ! memoire insufisante sdcard
		* 	0.0.3 - stats de mesure en pas can 
		* 	0.0.4 - suppression enregistrement sdcard 
		* 	0.0.5 - initialisation des moyennes (heure et 24 heures) à la valeur de la 1ere mesure
		* 	modification de la carte suppression de la commande de puissance du frigo
---------------------------------------------------------------------------------------------------
d		* 	0.1.0 - integation
		*	0.2.1 - mise en eprom des parametres

	*/

// programme :
const char title[] = "Acq_data_I-stat";
const char version[] = "0.1.0";

#include <Arduino.h>
#include <string.h>
#include <I2C_Anything.h>
#include "Timer.h"
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC
#include <Time.h>         //http://www.arduino.cc/playground/Code/Time  
#include <Wire.h>         //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <Streaming.h>        //http://arduiniana.org/libraries/streaming/

using namespace std;

/*	recapitulatif des i/o arduino
	#define rx		D0			// tty
	#define tx		D1			// tty
	#define 		D2			//
	#define 		D3			//
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

#define ANALOG_COUNT 2

const uint8_t ledPin = 8;			// the number of the LED pin
const uint8_t cdePin = 9 ;		// numero de sortie de commande

const unsigned int BAUD_RATE = 19200 ;

// parametres i2c
const uint8_t MY_ADDRESS = 12;
//const int MASTER_ADDRESS = 11;
// nombre d acquisition par minute
uint8_t s_freq = 10 ;				// nombre d acquisition par minute periode 6s = 10

	// definition des parametres d entrees analogiques ( 1 voie)
//const byte analog_pin[ANALOG_COUNT] = {0, 3} ; //, 2, 1};
const uint8_t analog_pin[ANALOG_COUNT] = {0, 2} ; //, 3, 1};
const char* analog_name[ANALOG_COUNT] = {"VBat", "IBat" } ; //, "I_ext", "opt"};
const char* ia_name[ANALOG_COUNT] = {"Am", "Ah" } ; //, "1ia_Min", "1iaH_moy"};
const int analog_IMIN[ANALOG_COUNT] = { 0, -22250 } ; //, -16, -16};
const int analog_IMAX[ANALOG_COUNT] = { 15000, 24000 } ; //, 16, 16};
long analog_value[ANALOG_COUNT] = {};
int offset[ANALOG_COUNT] = {0, 0} ;		// offset pascan conpensation ecart a 0amp
int gain[ANALOG_COUNT] = {100, 100} ;		// * gain de correction ana / 100  (voie courant negative)

// Variables :
bool debug = false ;   					// mode debug
bool sflag = false ;   					// caractere 's' reçu => mise a l heure
/**uint16_t inh[ANALOG_COUNT] [24] ;	// registre stat de data heure
	long iam[2] [60] ;				// puissance moyenne / minute pour calculer la moyenne glissante
	long iah[2] [24] ;					// puissance moyenne glissante en watt/heure
	long iamsigma[2] ;					// somme des puissances/minutes
	** utilisé pour 2 voie de calcul de puissance */
int inm[ ANALOG_COUNT ] ;		// registre stat de data minute : moyenne ( cumul / s_freq )
int iam[60] = {0};				// courant moyen / minute pour calculer la moyenne glissante
int iah[24] = {0};					// courant moyen glissant en ampere/heure
long iamsigma = 0 ;				// somme des puissances/minutes
const long maxiah = 200000 ;		// capacité max batterie
long cumuliah = 0 ;				// cumul energie
uint8_t cdeVal = 127;			// valeur de la periode de la commande au demarrage
bool init0 = true;
Timer timber;

// various commands we might get

enum {
	CMD_LIST_LENGTH = 1,
	CMD_LIST = 2,
	CMD_READ_VBAT = 3,
	CMD_READ_IBAT = 4,
	CMD_READ_IAM = 5,
	CMD_READ_IAH = 6,
	CMD_READ_CUMULIA = 7,
	CMD_ID = 9
	};

char command;

//			****	sous programmes		****

void(* resetFunc) (void) = 0; //declare reset function @ address 0

int freeRam () {
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
} // end of freeRam 

void ana_get(){			// acquisition analogiques
	for(int i=0;i<=ANALOG_COUNT-1;i++){
		long val = analogReadPin(analog_pin[i]);
		if(debug){
			Serial.print(val);
			Serial.print(F(" | "));
		}
		val = (val * gain[i]) / 100 ;
		if(debug){
			Serial.print(val);
			Serial.print(F(" | "));
		}
		if ( i == 1){		//	mesure capteur de courant
			val = map( val , 32, 1023, 480, -512);
		}
		
		val += offset[i];
		analog_value[i] +=  val;
		if(debug){
			Serial.print(val);
			Serial.print(F(" ; "));
			Serial.println();
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
	}
} // end of set_rtcTime

byte get_list(char *liste){		//get list of variables
	/*strcpy (liste, "Labels") ;
	strcat(liste, ",");
	strcat(liste, "TimeIndex");
	strcat(liste, ",");
	strcat(liste, "Date");
	strcat(liste, ",");
	*/
	for(int i= 0;i<=ANALOG_COUNT-1;i++){
		strcat(liste, analog_name[i]);
		strcat(liste, ",");
	}
	for(int i= 0;i<=ANALOG_COUNT-1;i++){
		strcat(liste, ia_name[i]);
		strcat(liste, ",");
	}
	strcat(liste, "cumulah");
	strcat(liste, ",");
	strcat(liste, "\n");
	return strlen(liste);
} // end of get_list

char* get_data(){		//get list of variables
	char liste [64];

	unsigned long t = now();
	
	sprintf(liste, "%ld", t);
	strcat(liste, ",");
	for(int i= 0;i<=ANALOG_COUNT-1;i++){
		strcat(liste, analog_name[i]);
		strcat(liste, ",");
	}
	for(int i= 0;i<=ANALOG_COUNT-1;i++){
		strcat(liste, ia_name[i]);
		strcat(liste, ",");
	}
	strcat(liste, "\n");
	return liste;
} // end of get_data

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

void receiveEvent (int howMany){
	command = Wire.read ();  // remember command for when we get request
} // end of receiveEvent

void requestEvent (){

	switch (command)
		{
	/**	CMD_LIST_LENGTH = 1,
		CMD_LIST = 2,
		CMD_READ_VBAT = 3,
		CMD_READ_IBAT = 4,
		CMD_READ_iaM = 5,
		CMD_READ_iaH = 6,
		CMD_READ_CUMULia = 7
		CMD_ID = 9
	*/
		case CMD_LIST_LENGTH : {
			char buffer[32] ;
			char* bfr = buffer ;
			byte rc = get_list(bfr) ;
			Wire.write (rc);					// send list length
			break;
		}
		case CMD_LIST : {
			char buffer[32] ;
			char* bfr = buffer ;
			byte rc = get_list(bfr) ;
			Wire.write (bfr);					// send list length
			break;
		}
		case CMD_READ_VBAT: send_ia (CMD_READ_VBAT); break;			// send VBAT
		case CMD_READ_IBAT: send_ia (CMD_READ_IBAT); break;			// send IBAT
		case CMD_READ_IAM: send_ia (CMD_READ_IAM); break;			// send iaM
		case CMD_READ_IAH: send_ia (CMD_READ_IAH); break;			// send iaH
		case CMD_READ_CUMULIA: send_ia (CMD_READ_CUMULIA); break;	// send CUMULia
		case CMD_ID: Wire.write (MY_ADDRESS);	break;				// send id
		}  // end of switch
	}  // end of requestEvent

void send_ia (byte valeur){

	if (valeur == CMD_READ_VBAT){
		I2C_writeAnything (inm[0]);
		if (debug){
			Serial.print(F("VBAT :"));
			Serial.print(inm[0]);
			Serial.print(F("\n"));
		}
	}
	else if (valeur == CMD_READ_IBAT){
		I2C_writeAnything (inm[1]);
		if (debug){
			Serial.print(F("IBAT :"));
			Serial.print(inm[1]);
			Serial.print(F("\n"));
		}
	}
	else if (valeur == CMD_READ_IAM){
		int t = now();
		int minut = minute(t);						// index minute actuelle
		I2C_writeAnything (iam[minut]);
		if (debug){
			Serial.print(F("IAM :"));
			Serial.print(iam[minut]);
			Serial.print(F("\n"));
		}
	}
	else if (valeur == CMD_READ_IAH){
		int heure = hour() ;
		I2C_writeAnything (iah[heure]);
		if (debug){
			Serial.print(F("IAH :"));
			Serial.print(iah[heure]);
			Serial.print(F("\n"));
		}
	}
	else if (valeur == CMD_READ_CUMULIA){
		I2C_writeAnything (cumuliah);
		if (debug){
			Serial.print(F("CUMUL :"));
			Serial.print(cumuliah);
			Serial.print(F("\n"));
		}
	}
} // end of send_ia

void print_title (){
	Serial.println();
	Serial.print(title);
	Serial.print(F(" "));
	Serial.println(version);
} // end of print_title

void print2serial(char *bfr, int iam, int iah){

	//Serial.print(F("freeRam : ")); 
	//Serial.println(freeRam()); 
	
	Serial.print(bfr) ;
	Serial.print(F(",")) ;
	// impression mesures de capteurs
	for(int i=0;i<=ANALOG_COUNT-1;i++){
		if ( i == 1){		//	mesure capteur de courant
			Serial.print( map(inm[i] , -512, 511, analog_IMIN[i], analog_IMAX[i]) );
		}
		else {				//	mesure de tension
			Serial.print( map(inm[i] , 0, 1023 , analog_IMIN[i], analog_IMAX[i]) );
		}
		Serial.print(F(",")) ;
	}
	/** impression sommes des acquisitions minutes glissantes
	Serial.print(map(iamsigma , 0, 61380, analog_IMIN[1], analog_IMAX[1]) ); 
	Serial.print(F(",")) ;**/
	// impression courant instantané minute
	Serial.print( map(iam , -512, 511, analog_IMIN[1], analog_IMAX[1]) );
	Serial.print(F(",")) ;
	// impression moyenne glissante du courant sur 60 minutes
	Serial.print( map(iah , -512, 511, analog_IMIN[1], analog_IMAX[1]) );
	Serial.print(F(",")) ;
	//impression du cumul de capacité
	Serial.print( cumuliah / 60) ;
	Serial.print(F("\n")) ;
	//Serial.print(F("freeRam : ")); 
	//Serial.println(freeRam()); 
}// end of print2serial

//			****	fin des sous programmes		****

void setup(void){

	command = 0;		// commande provenant i2c
	pinMode(ledPin, OUTPUT);
	digitalWrite(ledPin, HIGH);		// turn the LED on (HIGH is the voltage level)
	pinMode(cdePin, OUTPUT);
	digitalWrite(cdePin, LOW);		// turn the LED on (HIGH is the voltage level)
	//initialisation arrays
	ana_get() ;
	for(int j=0;j<=59;j++){			// raz de la moyenne glissante de puissance 
		iam [j] = 0 ;
	}
	iamsigma = 0;
	for(int i=0;i<=24-1;i++){
		iah[i] = analog_value[1];
	}
	Serial.begin(BAUD_RATE);		// initialize serial:
	print_title () ;
	setSyncProvider(RTC.get);
	if (timeStatus() != timeSet){
		Serial.print(F(" FAIL remote RTC!\n"));
	}
	Wire.begin (MY_ADDRESS);
	Wire.onReceive (receiveEvent);	// interrupt handler for incoming messages
	Wire.onRequest (requestEvent);	// interrupt handler for when data is wanted

	Serial.print(F("i2c en reception port :"));
	Serial.print(MY_ADDRESS);
	Serial.print(F("\n"));
	timber.oscillate(ledPin, 100, LOW, 10);
	time_t t;
	do{
		t = now();
		delay(1000);
		Serial.print(F("."));
	}while (second(t) != 0);
	Serial.print(F("\n"));
	Serial.print(F("Date Heure, Tension, Courant, ia_MoyMin,"));
	Serial.print(F(" ia_MoyGlHeure, ia_CumulTotal\n"));
	delay(500);

	timber.pulse(ledPin, 500, LOW);

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

				for(int i=0;i<=ANALOG_COUNT-1;i++){
					inm[i] = analog_value[i] / s_freq;
					analog_value[i] = 0 ;
					if(debug){
						Serial.print("inm[i] : ");
						Serial.print(inm[i]);
						Serial.print(" / ");
					}
				}
				int minut = minute(t);
				// capacité totale 60 minutes - ( ia vielle minute )
				iamsigma = iamsigma - iam [minut] ;
				// mise a jour capacité minute actuelle
				iam[minut] = inm[1] ;		// Ampère / minute
				// capacité totale 59 minutes + ( ia nouvelle minute )
				iamsigma = iamsigma + iam [minut] ;
				int heure = hour();
				//calcul capacité moyenne gissante horaire
				iah[heure] = iamsigma  / 60 ;
				//calcul capacité cumulée ah/mn
				cumuliah += iah[heure]; // ajout de la capacité minute
				if(debug){
					Serial.print(F("iamsigma : "));
					Serial.print(iamsigma );
					Serial.print(F(" | iah : "));
					Serial.print(iah[heure] );
					Serial.print(F(" | cumuliah : "));
					Serial.println(cumuliah );
					Serial.println() ;
					for(int i=0;i<=60-1;i++){
						Serial.print(F("|")) ;
						Serial.print(iam[i]);
					}
					Serial.println() ;
					for(int i=0;i<=24-1;i++){
						Serial.print(F("|")) ;
						Serial.print(iah[i]);
					}
					Serial.println() ;
				}
				char buffer[20] ;
				char *bfr = buffer ;
				digitalClockDisplay (bfr) ;
				print2serial(bfr, iam[minut], iah [heure]);
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
				Serial.print(F("b -> m/a frigo sur batterie\n"));
				Serial.print(F("d -> debug - reglage\n"));
				Serial.print(F("f -> frequence - fx\n"));
				Serial.print(F("l -> liste des variables\n"));
				Serial.print(F("g -> correction gain (gindex,x)\n"));
				Serial.print(F("o -> correction offset (oindex,x)\n"));
				Serial.print(F("p -> stat puissance 24H\n"));
				Serial.print(F("m -> stat puissance Horaire\n"));
				Serial.print(F("r -> reset avr\n"));
				Serial.print(F("syy,m,d,h,m,s -> mise a l heure\n"));
				Serial.print(F("v -> version\n"));
			}
			else if(octet == 's'){
				sflag = true;
				Serial.print(F("tapper la date au format yy,mm,dd,hh,mm,ss,\n"));
				if (Serial.available() >= 12){
					set_rtcTime();
				}
			}
			else if(octet == 'b'){
				delay(5);
				if (Serial.available() > 0){
					int val = Serial.parseInt();
					if (val == 0){
						digitalWrite(cdePin, LOW);
					}
					else if (val == 1){
						digitalWrite(cdePin, HIGH);
					}
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
			else if (octet == 'o'){			//set offset
				int index ;
				delay(5);
				 if (Serial.available() > 0){
					index = Serial.parseInt();
					Serial.print(index);
					Serial.print(F(", "));
					Serial.print(offset[index]);
					Serial.print(F(" : "));
					delay(5);
				}
				if (Serial.available() > 0){
					offset[index] = Serial.parseInt();
				}
				Serial.print(offset[index]);
				Serial.println();
			}
			else if (octet == 'g'){			//set gain
				int index ;
				delay(5);
				if (Serial.available() > 0){
					index = Serial.parseInt();
					Serial.print(index);
					Serial.print(F(", "));
					Serial.print(gain[index]);
					Serial.print(F(" : "));
					delay(5);
				}
				if (Serial.available() > 0){
					gain[index] = Serial.parseInt();
				}
				Serial.print(gain[index]);
				Serial.println() ;
			}
			else if (octet == 'r'){
				resetFunc() ;  //call reset
			}
			else if (octet == 'p'){
				for(int i=0;i<=24-1;i++){
					Serial.print(F("|")) ;
					Serial.print(iah[i]);
				}
				Serial.print(F("\n")) ;
			}
			else if (octet == 'm'){
				for(int i=0;i<=60-1;i++){
					Serial.print(F("|")) ;
					Serial.print(iam[i]);
				}
				Serial.println() ;
			}
			else if (octet == 'v'){
				print_title() ;
			}
			else {
				Serial.print(F("commande inconnue : "));
				Serial.print(octet);
				Serial.print(F(" tapper h pour l aide\n"));
			}
		}
	}
	else if (sflag){
		if (Serial.available() >= 12) {
			set_rtcTime();
		}
	}
} // end of serialEvent


