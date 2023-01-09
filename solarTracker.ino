/*         _          _____              _             
          | |        |_   _|            | |            
 ___  ___ | | __ _ _ __| |_ __ __ _  ___| | _____ _ __ 
/ __|/ _ \| |/ _` | '__| | '__/ _` |/ __| |/ / _ \ '__|
\__ \ (_) | | (_| | |  | | | | (_| | (__|   <  __/ |   
|___/\___/|_|\__,_|_|  \_/_|  \__,_|\___|_|\_\___|_|   

Autor: Finn Wattenbach und Martin Rösner
email: finn.wattenbach@gmx.de
email: roesner@elektronikschule.de
Version: Alpha
Lizenz: GPL-3.0
*/

//We always have to include the library
#include "LedControl.h"

/*
 Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
 pin 12 is connected to the DataIn 
 pin 11 is connected to LOAD(CS)
 pin 10 is connected to the CLK 
 We have only a single MAX72XX.
 */
LedControl lc = LedControl(12, 10, 11, 1);

// Motor Vertikal
#define VENABLE 5
#define VODIR 3
#define VUDIR 4
// Motor Horizontal
#define HENABLE 8
#define HLDIR 6
#define HRDIR 7
// LDR's
#define obenrechtsLDR 4
#define obenlinksLDR 3
#define untenrechtsLDR 6
#define untenlinksLDR 5
// Taster
#define horlinksTaster 22
#define horrechtsTaster 23
#define vertuntenTaster 24
#define vertobenTaster 25

#define pos_toleranz 10
#define neg_toleranz -10

#define matrix 13

/* image switching time */
unsigned long delaytime1 = 500;
unsigned long delaytime2 = 2000;
unsigned int  drehzeit = 50;
int i, ldr1, ldr2, ldrmax = 0;
int orldr, urldr, olldr, ulldr;
int orint, urint, olint, ulint, action;
bool voTaster, vuTaster, hlTaster, hrTaster;
byte pfeilLinks[8]       = { B00000000, B00010000, B00111000, B01111100, B00010000, B00010000, B00010000, B00000000 };
byte pfeilRechts[8]      = { B00000000, B00000000, B00010000, B00010000, B00010000, B01111100, B00111000, B00010000 };
byte pfeilUnten[8]       = { B00000000, B00000000, B00100000, B01100000, B11111100, B01100000, B00100000, B00000000 };
byte pfeilOben[8]        = { B00000000, B00000000, B00001000, B00001100, B01111110, B00001100, B00001000, B00000000 };
byte pfeilRechtsOben[8]  = { B00000000, B00000000, B00000000, B00100000, B00010000, B00001010, B00000110, B00001110 };
byte pfeilLinksUnten[8]  = { B00000000, B11100000, B11000000, B10100000, B00010000, B00001000, B00000000, B00000000 };
byte pfeilLinksOben[8]   = { B00000000, B00001110, B00000110, B00001010, B00010000, B00100000, B00000000, B00000000 };
byte pfeilRechtsUnten[8] = { B00000000, B00000000, B00000000, B00001000, B00010000, B10100000, B11000000, B11100000 };

void setup() {
  //--- Motor Vertikal ---
  pinMode(VENABLE, OUTPUT);
  pinMode(VODIR, OUTPUT);
  pinMode(VUDIR, OUTPUT);
  //--- Motor Horizontal ---
  pinMode(HENABLE, OUTPUT);
  pinMode(HLDIR, OUTPUT);
  pinMode(HRDIR, OUTPUT);
  // --- Taster
  pinMode(horlinksTaster, INPUT);   //22
  pinMode(horrechtsTaster, INPUT);  //23
  pinMode(vertuntenTaster, INPUT);  //24
  pinMode(vertobenTaster, INPUT);   //25
  //--- Monitor
  Serial.begin(9600);
  Serial.println("            _          _____              _             ");
  Serial.println("           | |        |_   _|            | |            ");
  Serial.println("  ___  ___ | | __ _ _ __| |_ __ __ _  ___| | _____ _ __ ");
  Serial.println(" / __|/ _ \\| |/ _` | \'__| | \'__/ _` |/ __| |/ / _ \\ \'__|");
  Serial.println(" \\__ \\ (_) | | (_| | |  | | | | (_| | (__|   <  __/ |   ");
  Serial.println(" |___/\\___/|_|\\__,_|_|  \\_/_|  \\__,_|\\___|_|\\_\\___|_|   ");
  Serial.println();

  // The MAX72XX is in power-saving mode on startup, we have to do a wakeup call
  lc.shutdown(0, false);
  // Helligkeit des Mediums setzen
  lc.setIntensity(0, matrix);
  // Display löschen
  lc.clearDisplay(0);
}

void solarTracker() {
  // Intensität als Spannungswert...
  orldr = analogRead(obenrechtsLDR);   //4
  urldr = analogRead(untenrechtsLDR);  //6
  olldr = analogRead(obenlinksLDR);    //3
  ulldr = analogRead(untenlinksLDR);   //5
  // Schalter für die Drehbereiche
  // Die Schalter befinden sich im High Zustand
  // sobald sie geschlossen sind gehen sie auf LOW
  voTaster = digitalRead(vertobenTaster);
  vuTaster = digitalRead(vertuntenTaster);
  hlTaster = digitalRead(horlinksTaster);
  hrTaster = digitalRead(horrechtsTaster);

  // Maximale Intensität bestimmen
  if (orldr >= urldr && orldr > ldrmax)
    ldrmax = orldr;
  else if (urldr >= orldr && urldr > ldrmax)
    ldrmax = urldr;

  ldrmax = 200;
  // Intensitäten aufgrund der maximalen Itensität
  // Berechnen
  float diff_oben   = orldr - olldr; // Horizontale Bewegung
  float diff_unten  = urldr - ulldr;
  float diff_links  = olldr - ulldr; // Vertikale Bewegung
  float diff_rechts = orldr - urldr;

  // Motor"stärke" einstellen
  //analogWrite(VENABLE,180);
  //analogWrite(HENABLE,180);
  digitalWrite(VENABLE, HIGH);
  digitalWrite(HENABLE, HIGH);
  /*
  Serial.print("ObenRechts  /  ObenLinks: ");
  Serial.print(orldr);
  Serial.print(" | ");
  Serial.println(olldr);
  Serial.print("UntenRechts / UntenLinks: ");
  Serial.print(urldr);
  Serial.print(" | ");
  Serial.println(ulldr);
  Serial.print("Oben/Unten: ");
  Serial.print(diff_oben);
  Serial.print(" | ");
  Serial.println(diff_unten);
  Serial.print("Links/rechts: ");
  Serial.print(diff_rechts);
  Serial.print(" | ");
  Serial.println(diff_links);*/
  // Horizontale Bewegung
  
  if (      (diff_oben < neg_toleranz && diff_unten < neg_toleranz) && (diff_rechts < neg_toleranz && diff_links < neg_toleranz) )
    action = 0;
  else if ( (diff_oben < neg_toleranz && diff_unten < neg_toleranz) && (diff_rechts > pos_toleranz && diff_links > pos_toleranz) )
    action = 1;
  else if ( (diff_oben > pos_toleranz && diff_unten > pos_toleranz) && (diff_rechts < neg_toleranz && diff_links < neg_toleranz) )
    action = 2;
  else if ( (diff_oben > pos_toleranz && diff_unten > pos_toleranz) && (diff_rechts > pos_toleranz && diff_links > pos_toleranz) )
    action = 3;
  else if ( diff_oben < neg_toleranz && diff_unten < neg_toleranz )
    action = 4;
  else if ( diff_oben > pos_toleranz && diff_unten > pos_toleranz )
    action = 5;
  else if ( diff_rechts < neg_toleranz && diff_links < neg_toleranz )
    action = 6;
  else if ( diff_rechts > pos_toleranz && diff_links > pos_toleranz )
    action = 7;
  // Switch case
  switch (action)  {
    case 0:
      // Bewegung nach rechts
      for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilRechtsUnten[m]);
      Serial.print(" <- ");
      Serial.print(diff_oben);
      Serial.print(" | ");
      Serial.println(diff_unten);
      digitalWrite(HRDIR, HIGH);  //one way
      digitalWrite(HLDIR, LOW);
      digitalWrite(VODIR, LOW);
      digitalWrite(VUDIR, HIGH);
      delay(drehzeit);
      digitalWrite(HRDIR, LOW);
      digitalWrite(VUDIR, LOW);
      break;
    case 1:
      // Bewegung nach rechts
      for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilRechtsOben[m]);
      Serial.print(" <- ");
      Serial.print(diff_oben);
      Serial.print(" | ");
      Serial.println(diff_unten);
      digitalWrite(HRDIR, HIGH);  //one way
      digitalWrite(HLDIR, LOW);
      digitalWrite(VODIR, HIGH);
      digitalWrite(VUDIR, LOW);
      delay(drehzeit);
      digitalWrite(VODIR, LOW);
      digitalWrite(HRDIR, LOW);
      break;
    case 2:
      //Bewegung nach links
      for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilLinksUnten[m]);
      Serial.print(" -> ");
      Serial.print(diff_oben);
      Serial.print(" | ");
      Serial.println(diff_unten);
      digitalWrite(HRDIR, LOW);  //reverse
      digitalWrite(HLDIR, HIGH);
      digitalWrite(VODIR, LOW);
      digitalWrite(VUDIR, HIGH);
      delay(drehzeit);
      digitalWrite(HLDIR, LOW);
      digitalWrite(VUDIR, LOW);
      break;
    case 3:
      //Bewegung nach links
      for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilLinksOben[m]);
      Serial.print(" -> ");
      Serial.print(diff_oben);
      Serial.print(" | ");
      Serial.println(diff_unten);
      digitalWrite(HRDIR, LOW);  //reverse
      digitalWrite(HLDIR, HIGH);
      digitalWrite(VODIR, HIGH);
      digitalWrite(VUDIR, LOW);
      delay(drehzeit);
      digitalWrite(HLDIR, LOW);
      digitalWrite(VODIR, LOW);
      break;
    case 4:
      // Bewegung nach rechts
      for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilRechts[m]);
      Serial.print(" <- ");
      Serial.print(diff_oben);
      Serial.print(" | ");
      Serial.println(diff_unten);
      digitalWrite(HRDIR, HIGH);  //one way
      digitalWrite(HLDIR, LOW);
      delay(drehzeit);
      digitalWrite(HRDIR, LOW);
      break;
    case 5:
      //Bewegung nach links
      for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilLinks[m]);
      Serial.print(" -> ");
      Serial.print(diff_oben);
      Serial.print(" | ");
      Serial.println(diff_unten);
      digitalWrite(HRDIR, LOW);  //reverse
      digitalWrite(HLDIR, HIGH);
      delay(drehzeit);
      digitalWrite(HLDIR, LOW);/* code */
      break;
    case 6:
      // Bewegung nach unten
      for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilUnten[m]);
      Serial.print(" v  ");
      Serial.print(diff_rechts);
      Serial.print(" | ");
      Serial.println(diff_links);
      digitalWrite(VUDIR, HIGH);
      digitalWrite(VODIR, LOW);
      delay(drehzeit);
      digitalWrite(VUDIR, LOW);
      break;
    case 7:
      //Bewegung nach oben
      for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilOben[m]);
      Serial.print(" ^  ");
      Serial.print(diff_links);
      Serial.print(" | ");
      Serial.println(diff_rechts);
      digitalWrite(VODIR, HIGH);
      digitalWrite(VUDIR, LOW);
      delay(drehzeit);
      digitalWrite(VODIR, LOW);
      break;
    default:
      digitalWrite(HRDIR, LOW);
      digitalWrite(HLDIR, LOW);
      delay(100);
  }
 
/*
  GrimsGrams
  for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilLinks[m]);
  for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilLinksUnten[m]);
  for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilUnten[m]);
  for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilRechtsUnten[m]);
  for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilRechts[m]);
  for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilRechtsOben[m]);
  for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilOben[m]);
  for (int m = 0; m <= 7; m++) lc.setRow(0, m, pfeilLinksOben[m]);
*/
  delay(500);
  lc.clearDisplay(0);
}


void loop() {
  solarTracker();
}
