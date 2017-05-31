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
M		* 	0.0.6 - calcul moyenne glissante = somme -vieux + nouveau / nombre
		* 	0.0.7 - envoi de donnee modele csv
		* 	0.0.8 - envoi de float sur wire
D		* 	0.0.9 - correction A/h au lieu de W/h 
	*/

// programme :
const char title[] = "Acq_data_U-I-P";
const char version[] = "0.0.9";

#include <Arduino.h>
#include "Timer.h"
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC
#include <Time.h>         //http://www.arduino.cc/playground/Code/Time  
#include <Wire.h>         //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <Streaming.h>        //http://arduiniana.org/libraries/streaming/
#include <I2C_Anything.h>


/*	recapitulatif des i/o arduino
	#define rx	D0			// libre
	#define tx	D1			// libre
	#define 		D2			// libre
	#define 		D3			// libre
	#define 		D4			// libre
	#define 		D5			// libre
	#define 		D6			// libre
	#define 		D7			// libre
	#define 		D8			// ledpin
	#define 		D9			// libre
	#define 		D10			// libre
	#define mosi	D11			// SPI SD	MOSI
	#define miso	D12			// SPI SD	MISO
	#define sck 	D13			// SPI SD	SCK
	#define 		A0			// I mes1
	#define 		A1			// I mes2
	#define 		A2			// I mes3
	#define 		A3			// V_25v
	#define sda	A4			// I2C rtc
	#define scl	A5			// I2C rtc
*/

//#define ANALOG_COUNT 4		// utilisation des 4 canaux de mesure
#define ANALOG_COUNT 2

const int ledPin = 8;			// the number of the LED pin
const unsigned int BAUD_RATE = 19200 ;

// parametres i2c
const int MY_ADDRESS = 12;
const int MASTER_ADDRESS = 11;

	// definition des parametres d entrees analogiques ( 1 voie)
char* analog_name[ANALOG_COUNT] = {"VBat", "IBat" } ; //, "I_ext", "opt"};
char* ia_name[ANALOG_COUNT] = {"Am", "Ah" } ; //, "1ia_Min", "1iaH_moy"};
//const byte analog_pin[ANALOG_COUNT] = {0, 3} ; //, 2, 1};
const byte analog_pin[ANALOG_COUNT] = {0, 2} ; //, 3, 1};
	//les nombres sont à multiplier par 2puissance23 pour travailler avec des entiers
const long analog_IMIN[ANALOG_COUNT] = { 0, -16000 } ; //, -16, -16};
const long analog_IMAX[ANALOG_COUNT] = { 15000, 16000 } ; //, 16, 16};
long analog_value[ANALOG_COUNT];
int offset[ANALOG_COUNT] = {0, -105} ;		// offset pascan conpensation ecart a 0amp
int gain[ANALOG_COUNT] = {115, 115} ;		// gain de correction ana
//const float bandgap_voltage = 1.1 ;	// this is not super guaranteed but its not -too- off
	//float vref = 12.600 ;		// tension alim 25.24/2  actuellement non utilisé
	//const int refcan = 512 ;		// pas can a demi plage 0amp actuellement non utilisé

// Variables :
bool sflag;   					// caractere 's' reçu => mise a l heure
int s_freq = 10 ;				// nombre d acquisition par minute periode 6s = 10
//long ins[ ANALOG_COUNT ] ;		// registre stat de data 60 / s_freq = toute les n secondes
long inm[ ANALOG_COUNT ] ;		// registre stat de data minute : moyenne ( cumul / s_freq )
long inh[ANALOG_COUNT] [24] ;	// registre stat de data heure
	/*float iam[2] [60] ;				// puissance moyenne / minute pour calculer la moyenne glissante
	float iah[2] [24] ;					// puissance moyenne glissante en watt/heure
	float iamsigma[2] ;					// somme des puissances/minutes
	** utilisé pour 2 voie de calcul de puissance */
long iam[60] ;					// puissance moyenne / minute pour calculer la moyenne glissante
long iah[24] ;					// puissance moyenne glissante en watt/heure
long iamsigma ;				// somme des puissances/minutes
const int maxiah = 200000 ;		// capacité max batterie
long cumuliah ;				// cumul energie
int minut ;						// index minute actuelle
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

/** Mesure entree ANA avec la référence interne à 1.1 volts */
unsigned int analogReadPin(byte pin) {

	/** Sélectionne l entee ADC avec la référence interne à 1.1 volts */
	//ADMUX = bit (REFS0) | bit (REFS1)  | pin;    // input pin
	ADMUX = bit (REFS0) | pin ;    // input pin
	delay (200);  // let it stabilize
	/** Active le convertisseur analogique -> numérique **/
	ADCSRA |= (1 << ADEN);
	/** Lance une conversion analogique -> numérique **/
	ADCSRA |= (1 << ADSC);
	/** Attend la fin de la conversion **/
	while(ADCSRA & (1 << ADSC));
	/** Récupère le résultat de la conversion **/
	return ADCL | (ADCH << 8);
}

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

byte send_labels(char *liste){		//send list of variables
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
} // end of send_list

char* send_data(){		//send list of variables
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
} // end of send_list

char* digitalClockDisplay(char* buffer){
	int jour = day();
	int mois = month();
	int annee = year();
	int heure = hour();
	int minutes = minute();
	int secondes = second();
	sprintf(buffer,"%02d:%02d:%02d %02d/%02d/%04d", heure ,minutes ,secondes, jour, mois, annee);  //  %d pour un int
//  reporter le date et l heure à la demande après la trame de donnees RT
//printf(buffer,"Date\t%02d/%02d/%04d", jour, mois, annee);  //  %d pour un int
//	console.println(buffer);
//	sprintf(buffer,"Heure\t%02d:%02d:%02d", heure ,minutes ,secondes);  //  %d pour un int
	//Serial.print(buffer);
	return buffer ;
}

void analog_convert(){	//conversion de 10 acquisitions pascan / minutes

	for(int i=0;i<=ANALOG_COUNT-1;i++){
		if ( i> 0){
			inm[i] = map(analog_value[i] , 0, 10240, analog_IMAX[i], analog_IMIN[i]) * gain[i] / 100;
		}
		else {
			inm[i] = map(analog_value[i] , 0, 10240, analog_IMIN[i], analog_IMAX[i]);
		}
		Serial.println(inm[i]) ;

	}
} // end of analog_convert 

void ana_get(){			// acquisition analogiques
	for(int i=0;i<=ANALOG_COUNT-1;i++){
		int val = analogReadPin(analog_pin[i]) + offset[i];
		analog_value[i] += val ;
		Serial.print(val) ;
		Serial.print(" | ") ;
	}
	Serial.println() ;

} // end of ana_get 

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
			byte rc = send_labels(bfr) ;
			Wire.write (rc);					// send list length
			break;
		}
		case CMD_LIST : {
			char buffer[32] ;
			char* bfr = buffer ;
			byte rc = send_labels(bfr) ;
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
	}
	else if (valeur == CMD_READ_IBAT){
		I2C_writeAnything (inm[1]);
	}
	else if (valeur == CMD_READ_IAM){
		I2C_writeAnything (iam[minut]);
	}
	else if (valeur == CMD_READ_IAH){
		int heure = hour() ;
		I2C_writeAnything (iah[heure]);
	}
	else if (valeur == CMD_READ_CUMULIA){
		I2C_writeAnything (cumuliah);
	}

} // end of send_ia

//			****	fin des sous programmes		****

void setup(void){

	command = 0;
	pinMode(ledPin, OUTPUT);
	digitalWrite(ledPin, HIGH);		// turn the LED on (HIGH is the voltage level)
	//for(int i=0;i<=1;i++){
		for(int j=0;j<=59;j++){			// raz de la moyenne glissante de puissance 
			iam [j] = 0 ;
		}
	//}
	Serial.begin(BAUD_RATE);		// initialize serial:
	Serial.println();
	Serial.print(title);
	Serial.print(F(" "));
	Serial.println(version);
	//setSyncProvider() causes the Time library to synchronize with the
	//external RTC by calling RTC.get() every five minutes by default.
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

} // end of setup

void loop(void){
	static time_t tLast;
	time_t t;

	if (sflag == false){
		t = now();
		if (t != tLast) {
			tLast = t;
			if (second(t) % (60/s_freq) == 0 ) {		//acquisition des mesures
				timber.pulse(ledPin, 500, LOW);
				ana_get() ;
				/*for(int i=0;i<=ANALOG_COUNT-1;i++){
					ins[i] += analog_value[i] ;		// cumul des x acquisitions / minute
					//Serial.print(ins[i]) ;
					//Serial.print(" | ") ;
					
				}*/
				//Serial.println() ;
			}
			if (second(t) == 0) {						// calculs des mesures moyenne / minute
				//Serial.print(F("time: ")) ;
				//Serial.print(now()) ;
				//Serial.print(",") ;
				char buffer[20] ;
				char* bfr = buffer ;
				digitalClockDisplay (bfr) ;
				Serial.print(buffer) ;
				Serial.print(F(",")) ;
				//Serial.print(F("calcul de la minute : \n")) ;
				analog_convert() ; 
				for(int i=0;i<=ANALOG_COUNT-1;i++){
					//inm[i] = ins[i]  ;			// acquisition minute = cumul
					Serial.print(inm[i]) ;
					Serial.print(F(",")) ;
					analog_value[i] = 0 ;
				}
				//Serial.print(F("  ")) ;
				/*for(int i=1;i<=2;i++){			// calcul de la puissance moyenne / minute
					iam[i -1] [minute(t)] = inm[0] * inm[1];
					Serial.print("|") ;
					Serial.print(iam[i -1] [minute(t)]);
					Serial.print("|") ;
				}
				Serial.print("\n") ;
				*/
				/**for(int i=0;i<=1;i++){
					// puissance moyenne / horaire - ( ia vielle minute )
					iamsigma[i] = iamsigma[i] - iam[i] [minute(t)] ;
					// calcul de la puissance moyenne / nouvelle minute
					iam[i] [minute(t)] = inm[0] * inm[i +1];
					Serial.print(F(" |ia/min= ")) ;
					Serial.print(iam[i] [minute(t)]);
					// puissance moyenne / horaire + ( ia nouvelle minute )
					iamsigma[i] = iamsigma[i] + iam[i] [minute(t)] ;
					iah[i] [hour()] = iamsigma[i] / 60 ;
					Serial.print(F(" |iamoy/h= ")) ;
					Serial.print(iamsigma[i] /60) ;
				}*/
				// puissance moyenne / horaire - ( ia vielle minute )
				minut = minute(t);
				iamsigma = iamsigma - iam [minut] ;
				// calcul de la puissance moyenne / nouvelle minute
				///iam [minut] = inm[0] * inm[1];		// produit = tension * courant
				iam [minut] = inm[1];		// Ampère / minute
				//Serial.print(F(" |ia/min= ")) ;
				Serial.print(iam [minut]);
				Serial.print(F(",")) ;
					// puissance moyenne / horaire + ( ia nouvelle minute )
				iamsigma = iamsigma + iam [minut] ;
				int heure = hour();
				iah [heure] = iamsigma / 60 ;
				//Serial.print(F(" |iamoy/h= ")) ;
				Serial.print(iah [heure]) ;
				Serial.print(F(",")) ;
				cumuliah += iam [minut] / 60 ;
				if (cumuliah > maxiah){
					cumuliah = maxiah ;
				}
				Serial.print(cumuliah) ;
				Serial.print(F(",\n")) ;
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
				Serial << F("tapper : \n"
							"h ou ? -> ce menu\n"
							"syy,m,d,h,m,s -> mise a l heure\n"
							"l -> liste des variables\n"
							"o -> correction offset courant\n"
							"g -> gain courant\n"
							"p -> stat puissance 24H\n"
							"r -> reset avr\n") << endl;
			}
			else if(octet == 's'){
				sflag = true;
				Serial << F("tapper la date au format yyyy,mm,dd,hh,mm,ss,\n") << endl;
				if (Serial.available() >= 12){
					set_rtcTime();
				}
			}
			else if(octet == 'l'){
				char buffer[63] ;
				char* bfr = buffer ;
				send_labels(bfr);
				Serial.print(buffer) ;
			}
			else if (octet == 'o'){			//set offset
				int index ;
				delay(10);
				 while (Serial.available() > 0){
					index = Serial.parseInt();
					Serial.print(offset[index]);
					Serial.print(" : ");
					offset[index] = Serial.parseInt();
				 }
				Serial.print(offset[index]);
				Serial.println();
			}
			else if (octet == 'g'){			//set gain
				int index ;
				delay(10);
				while (Serial.available() > 0){
					int index = Serial.parseInt();
					Serial.print(gain[index]);
					Serial.print(" : ");
					gain[index] = Serial.parseInt();
				}
				Serial.print(gain[index]);
				Serial.println();
			}
			else if (octet == 'r'){
				 resetFunc();  //call reset
			}
			else if (octet == 'p'){
				for(int i=0;i<=24-1;i++){
					Serial.print("|") ;
					Serial.print(iah[i]);
				}
				Serial.print("\n") ;
			}
			else {
				Serial.print(F("commande inconnue : '"));
				Serial.print(octet);
				Serial.print(F("' tapper h pour l aide\n"));
			}
		}
	}
	else if (sflag){
		if (Serial.available() >= 12) {
			set_rtcTime();
		}
	}
} // end of serialEvent



