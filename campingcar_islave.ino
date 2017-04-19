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
		* 
	*/

// programme :
const char title[] = "Acq data U-I-P";
const char version[] = "0.2.0";

#include <Arduino.h>
#include "Timer.h"
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC
#include <Time.h>         //http://www.arduino.cc/playground/Code/Time  
#include <Wire.h>         //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <Streaming.h>        //http://arduiniana.org/libraries/streaming/

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

const int ledPin = 8;			// the number of the LED pin
const unsigned int BAUD_RATE = 19200 ;

// parametres i2c
const byte MY_ADDRESS = 12;
const byte MASTER_ADDRESS = 11;

// definition des parametres d entrees analogiques
const int ANALOG_COUNT = 4;		// taille de la table de variables ana
char* analog_name[ANALOG_COUNT] = {"V_ANA", "I_bat_cell", "I_bat_in", "i_aux"};
const int analog_pin[ANALOG_COUNT] = {0, 3, 2, 1};
//les nombres sont multipliée par 100000 pour travailler avec des entiers
const int analog_IMIN[ANALOG_COUNT] = { 0, -15, -15, -15};
const int analog_IMAX[ANALOG_COUNT] = { 15, 15, 15, 15};
float analog_value[ANALOG_COUNT];
const float bandgap_voltage = 1.1 ;		// this is not super guaranteed but its not -too- off
const float vref = 12.62 ; // tension alim 25.24/2 ^^

// Variables :
bool sflag;   // caractere 's' reçu => mise a l heure
Timer timber;

//			****	sous programmes		****

void(* resetFunc) (void) = 0; //declare reset function @ address 0

/* Mesure la référence interne à 1v1 de l'ATmega */
unsigned int getInternal_1v1(void){
	ADMUX = 0x4E;					// Sélectionne la référence interne à 1v1 comme point de mesure, avec comme limite haute VCC
	ADCSRA |= (1 << ADEN);			// Active le convertisseur analogique -> numérique
	ADCSRA |= (1 << ADSC);			// Lance une conversion analogique -> numérique
	while(ADCSRA & (1 << ADSC));	// Attend la fin de la conversion
	return ADCL | (ADCH << 8);		// Récupère le résultat de la conversion
}

//print date and time to Serial
void printDateTime(time_t t){
	printDate(t);
	Serial << ' ';
	printTime(t);
} // end of printDateTime

//print time to Serial
void printTime(time_t t){
	printI00(hour(t), ':');
	printI00(minute(t), ':');
	printI00(second(t), ' ');
} // end of printTime

//print date to Serial
void printDate(time_t t){
	printI00(day(t), 0);
	Serial << monthShortStr(month(t)) << _DEC(year(t));
} // end of printDate

//Print an integer in "00" format (with leading zero),
//followed by a delimiter character to Serial.
//Input value assumed to be between 0 and 99.
void printI00(int val, char delim){
	if (val < 10) Serial << '0';
	Serial << _DEC(val);
	if (delim > 0) Serial << delim;
	return;
} // end of printI00

//set rtc time
void set_rtcTime(){
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
			//printDateTime(t);
			digitalClockDisplay() ;
			Serial << endl;
			//dump any extraneous input
			while (Serial.available() > 0) Serial.read();
		}
	}
} // end of set_rtcTime

//send list of variables
char* send_list(){
	char liste [30];
	strcpy (liste, analog_name[0]) ;
	for(int i= 1;i<=ANALOG_COUNT-1;i++){
		strcat(liste, analog_name[i]);
		strcat(liste, ",");
	}
	strcat(liste, "\n");
	return liste;
} // end of send_list

void digitalClockDisplay(void){
	char buffer[27] ;
	int jour = day();
	int mois = month();
	int annee = year();
	int heure = hour();
	int minutes = minute();
	int secondes = second();
	sprintf(buffer,"Date : %02d:%02d:%02d %02d/%02d/%04d", heure ,minutes ,secondes, jour, mois, annee);  //  %d pour un int
//  reporter le date et l heure à la demande après la trame de donnees RT
//printf(buffer,"Date\t%02d/%02d/%04d", jour, mois, annee);  //  %d pour un int
//	console.println(buffer);
//	sprintf(buffer,"Heure\t%02d:%02d:%02d", heure ,minutes ,secondes);  //  %d pour un int
	Serial.println(buffer);
}

/*float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/

void analog_get(){

	unsigned int val;
	float variable;

	val = getInternal_1v1() ;
	// variable affecte a real_vcc
	variable = 1023 * bandgap_voltage / val ;

	for(int i=0;i<=ANALOG_COUNT-1;i++){

		float analog_coef = 1023 / (analog_IMAX[i] - analog_IMIN[i]) * variable;
		val = analogRead(analog_pin[i]);
		//float analog_val = val * variable / analog_coef;
		//analog_value[i] = map(val, 0, 1023, analog_IMIN[i], analog_IMAX[i]);
		//float analog_val1 = mapfloat(val, 0, 1023, analog_IMIN[i], analog_IMAX[i]);
		if ( i> 0){
			analog_value[i] = (vref - (val * variable / analog_coef)) / 3 ;
		}
		else {
			analog_value[i] = val * variable / analog_coef ;
		}
		Serial.print(analog_name[i]);
		Serial.print(" : ");
		Serial.print(val) ;
		Serial.print(" : ");
		Serial.print(analog_value[i]);
		//Serial.print(" | ");
		//Serial.print(analog_val);
		//Serial.print(" | ");
		//Serial.print(analog_val1);
		Serial.print("\n");
	}
} // end of analog_get 

void receiveEvent (int howMany){
	for (int i = 0; i < howMany; i++){
		byte b = Wire.read ();
		Serial.write(b);
		if(b == 'l'){
			Wire.write (send_list());
		}
		else {
			Serial.print(F("commande i2c inconnue : "));
			Serial.print(b);
			Serial.print(F("\n"));
		}

	}
} // end of receiveEvent

//			****	fin des sous programmes		****

void setup(void){
	pinMode(ledPin, OUTPUT);
	digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
	// initialize serial:
	Serial.begin(BAUD_RATE);
	Serial.println();
	Serial.print(title);
	Serial.print(F(" "));
	Serial.println(version);
	//setSyncProvider() causes the Time library to synchronize with the
	//external RTC by calling RTC.get() every five minutes by default.
	setSyncProvider(RTC.get);
	//Serial << F("RTC Sync");
	if (timeStatus() != timeSet){
		Serial.print(F(" FAIL remote RTC!\n"));
	}
	Wire.begin (MY_ADDRESS);
	Wire.onReceive (receiveEvent);
	Serial.print(F("i2c en reception port :"));
	Serial.print(MY_ADDRESS);
	Serial.print(F("\n"));
	digitalWrite(ledPin, LOW);   // turn the LED off (LOW is the voltage level)
	timber.oscillate(ledPin, 100, LOW, 10);

} // end of setup

void loop(void){
	static time_t tLast;
	time_t t;
	tmElements_t tm;

	if (sflag == false){
		t = now();
		if (t != tLast) {
			tLast = t;
			if (second(t) % 5 == 0 ) {
			//if (second(t) == 0 or second(t) == 20 or second(t) == 40) {
				//printDateTime(t);
				Serial.println() ;
				digitalClockDisplay ();
				//timber.oscillate(ledPin, 500, LOW, 2);
				timber.pulse(ledPin, 500, LOW);
				/*float c = RTC.temperature() / 4.;
				float f = c * 9. / 5. + 32.;
				Serial << F("  ") << c << F(" C  ") << f << F(" F");*/
				//Serial << endl;
				analog_get() ;
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
				Serial.print(send_list());
			}
			else if (octet == 'r'){
				 resetFunc();  //call reset
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



