#include <EEPROM.h>

// Include the 7 seg LED display library:
#include <TM1637Display.h>

#define buzzerPin A4 
#define relayPin A5

//Define thermistor analogic pins
#define tubeTempPin 0  //A0
#define waterTempPin 1 //A1

// define Water Flow Sensor Data pin
#define wfsPin 2 //2 

// Define 7 seg LED display connections pins:
#define clkPin 3 //D3
#define dioPin 4 //D4

// define capacitive touch sensors pins
#define ctsPinUP 5   //D5 Pin for capactitive touch sensor +/up
#define ctsPinDOWN 6 //D6 Pin for capactitive touch sensor -/down

// define set/read switch
#define swPin 7 //D7

// define LED pinout
#define fluxAlarmPin 8    //D8
#define waterAlarmPin 9   //D9
#define tubeAlarmPin  10  //D10
#define fluxSelectPin 11  //D11
#define waterSelectPin 12 //D12
#define tubeSelectPin  13 //D13

#define AVERAGEVECTORSIZE 25 // length of the average vector used to smooth analog reading

// Create display object of type TM1637Display:
TM1637Display display = TM1637Display(clkPin, dioPin);

// Create array that turns all segments on:
const uint8_t AllOn[] = {0xff, 0xff, 0xff, 0xff};

// Create array for string display
const uint8_t STRinit[] = {
  SEG_C ,                                 // i
  SEG_C | SEG_E | SEG_G ,                 // n
  SEG_C ,                                 // i
  SEG_D | SEG_E | SEG_F | SEG_G           // t
};

const uint8_t STRhelo[] = {
  SEG_C | SEG_E | SEG_F | SEG_G ,          // h
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,   // e
  SEG_D | SEG_E | SEG_F ,                  // l
  SEG_C | SEG_D | SEG_E | SEG_G            // O
};

// Create degree Celsius symbol:
const uint8_t STRcelsius[] = {
  SEG_A | SEG_B | SEG_F | SEG_G,  // Circle
  SEG_A | SEG_D | SEG_E | SEG_F   // C
};

// Create FLow symbol:
const uint8_t STRflow[] = {
  SEG_D | SEG_E | SEG_F                    // L
};

// flow reading
volatile int wfsPulses; // Measures flow sensor pulses
float l_min; // water flow

// capacitive touch sensors
int ctsPreviousUP=LOW;
int ctsValueUP = LOW;
int ctsPreviousDOWN=LOW;
int ctsValueDOWN = LOW;
int swPosition=0;

// Temp reading
int Vo;
float logR2, R2, TubeTemp, WaterTemp;

// temp smoothing
int WaterTempReadIndex = 0;              // the index of the current reading
float WaterTempTotal = 0;                  // the running total
float WaterTempAverage = 0;                // the average
int TubeTempReadIndex = 0;              // the index of the current reading
float TubeTempTotal = 0;                  // the running total
float TubeTempAverage = 0;                // the average

// Tube temp params
float tubeR1 = 10000;
float tubec1 = 1.009249522e-03, tubec2 = 2.378405444e-04, tubec3 = 2.019202697e-07;

//water temp params
float waterR1 = 10000;
float waterc1 = 1.009249522e-03, waterc2 = 2.378405444e-04, waterc3 = 2.019202697e-07;

// setting values vector [0]=FLOW, [1]=Water temp, [2]=Tube temp
#define FLOW 0
#define WATER 1
#define TUBE 2
int Settings[3]={110,30, 30};
int Alarms[3]={LOW, LOW, LOW};

float TubeTempV[ AVERAGEVECTORSIZE];
float WaterTempV[ AVERAGEVECTORSIZE];

// memorise cursor position
int cursor = 0;
int oldcursor = 0;

// snooze sound alarm
int snooze=LOW;

void setup() {

  // Get last saved values for settings
  EEPROM.get(0, Settings);

  // ======================================================================================================================================================
  // Paramétrage du timer1, pour qu'il déclenche une interruption, à chaque fois que sa valeur sera égale à celle qu'on aura indiqué dans le registre OCR1A  
  // ======================================================================================================================================================
  noInterrupts();                 // On désactive les interruptions, pour commencer

  // On règle les bits WGM10, WGM11, WGM12, WGM13 pour fonctionner en mode "CTC" (c'est à dire, en mode "comparaison timer <-> valeur de référence")
  bitClear(TCCR1A, WGM10);        // On met le bit WGM10 à 0 (contenu dans le registre TCCR1A)
  bitClear(TCCR1A, WGM11);        // On met le bit WGM11 à 0 (contenu dans le registre TCCR1A)
  bitSet(TCCR1B, WGM12);          // On met le bit WGM12 à 1 (contenu dans le registre TCCR1B)
  bitClear(TCCR1B, WGM13);        // On met le bit WGM13 à 0 (contenu dans le registre TCCR1B)

  // Ensuite, on règle les bits CS10, CS11, et CS12 pour que le prédiviseur du timer1 fasse une division par 256
  bitClear(TCCR1B, CS10);         // On met le bit CS10 à 0 (contenu dans le registre TCCR1B)
  bitClear(TCCR1B, CS11);         // On met le bit CS11 à 0 (contenu dans le registre TCCR1B)
  bitSet(TCCR1B, CS12);           // On met le bit CS12 à 1 (contenu dans le registre TCCR1B)

  // Puis on active l'interruption du timer1, qui test en permanence s'il y a égalité entre la valeur courant du timer, et la valeur
  // stockée dans un registre de comparaison. En pratique, pour ce faire, on va mettre à 1 le bit OCIE1A dans le registre TIMSK1,
  // afin qu'une interruption soit générée, à chaque fois que la valeur du timer1 sera égale à la valeur qu'on aura renseigné dans le registre OCR1A
  bitSet(TIMSK1, OCIE1A);         // On met le bit OCIE1A à 1 (contenu dans le registre TIMSK1)

   // Enfin, on met le compteur à zéro, on entre la valeur déclenchant l'interruption (nos "31250" coups d'horloge), et on réactive les interruptions
  TCNT1 = 0;            // Mise du timer1 à zéro
  // OCR1A = 62500;        // Valeur correspondant à 1 seconde (car 62500 fois 16 µS donne bien 1 sec ; pour rappel, ces 16 µS proviennent du calcul 1/(16MHz/256),
                          // avec les 16 MHz correspondant à la fréquence du quartz de l'ATmega328P, et le 256 au réglage du prédiviseur du timer1 fait précédemment)
  OCR1A = 31250;       // interrupt toutes les 1/2 secondes

  pinMode(wfsPin, INPUT);
  digitalWrite(wfsPin, HIGH); // Optional Internal Pull-Up
  attachInterrupt(0, flow, RISING); // Setup Interrupt
 
  interrupts();         // Et, on ré-active les interruptions

  Serial.begin(9600);
  Serial.println( "start");

  pinMode( relayPin, OUTPUT);
  pinMode(ctsPinUP, INPUT);
  pinMode(ctsPinDOWN, INPUT);
  pinMode(swPin, INPUT_PULLUP);

  pinMode( fluxAlarmPin, OUTPUT);
  pinMode( waterAlarmPin, OUTPUT);
  pinMode( tubeAlarmPin, OUTPUT);
  pinMode( fluxSelectPin, OUTPUT);
  pinMode( waterSelectPin, OUTPUT);
  pinMode( tubeSelectPin, OUTPUT);
  
  // Clear the display:
  display.clear();
  delay(1000);
  display.setBrightness(4);
  display.setSegments(STRinit);
  delay(2000);
  display.setSegments(AllOn);
  delay(1000);
  display.clear();

  digitalWrite( fluxAlarmPin, HIGH); 
  delay( 200);
  digitalWrite( fluxAlarmPin, LOW);
  digitalWrite( waterAlarmPin, HIGH); 
  delay( 200);
  digitalWrite( waterAlarmPin, LOW);
  digitalWrite( tubeAlarmPin, HIGH); 
  delay( 200);
  digitalWrite( tubeAlarmPin, LOW);
  digitalWrite( fluxSelectPin, HIGH); 
  delay( 200);
  digitalWrite( fluxSelectPin, LOW);
  digitalWrite( waterSelectPin, HIGH); 
  delay( 200);
  digitalWrite( waterSelectPin, LOW);
  digitalWrite( tubeSelectPin, HIGH); 
  delay( 200);
  digitalWrite( tubeSelectPin, LOW);
  
  display.setSegments(STRhelo);
  delay(2000);
  display.clear();

  // initialize the average vector
  for (int i=0; i<AVERAGEVECTORSIZE; i ++) TubeTempV[ i] = WaterTempV[ i] = 0.0;
}

// ======================
// Routine d'interruption, appelées toutes les 1/2 secondes
// ======================
ISR(TIMER1_COMPA_vect) 
{
 l_min = (wfsPulses * 120.0 / 563.0);  // *120 pour obtenir des minutes (120 demi secondes) 
 wfsPulses = 0; // Reset Counter 
}
 
void flow () // Interrupt function
{
   wfsPulses++;
}

void loop() {
  // tube temp reading
  Vo = analogRead(tubeTempPin);
  R2 = tubeR1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  TubeTemp = (1.0 / (tubec1 + tubec2*logR2 + tubec3*logR2*logR2*logR2));
  TubeTemp = TubeTemp - 273.15;

  // smooth read value by evaluating the average value
  TubeTempTotal = TubeTempTotal - TubeTempV[TubeTempReadIndex];
  TubeTempV[TubeTempReadIndex] = TubeTemp;
  TubeTempTotal = TubeTempTotal + TubeTemp;
  TubeTempReadIndex = ( TubeTempReadIndex + 1 ) % AVERAGEVECTORSIZE;
  TubeTempAverage = TubeTempTotal / AVERAGEVECTORSIZE;
  
  // Water temp reading
  Vo = analogRead(waterTempPin);
  R2 = waterR1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  WaterTemp = (1.0 / (waterc1 + waterc2*logR2 + waterc3*logR2*logR2*logR2));
  WaterTemp = WaterTemp - 273.15;

  // smooth read value by evaluating the average value
  WaterTempTotal = WaterTempTotal - WaterTempV[WaterTempReadIndex];
  WaterTempV[WaterTempReadIndex] = WaterTemp;
  WaterTempTotal = WaterTempTotal + WaterTemp;
  WaterTempReadIndex = ( WaterTempReadIndex + 1 ) % AVERAGEVECTORSIZE;
  WaterTempAverage = WaterTempTotal / AVERAGEVECTORSIZE;

  // check if any alarm are trigerred, then switch on the related LED
  Alarms[TUBE] = (TubeTempAverage >= Settings[TUBE]);
  digitalWrite( tubeAlarmPin, (TubeTempAverage >= Settings[TUBE])); 
  Alarms[WATER] = (WaterTempAverage >= Settings[WATER]);
  digitalWrite( waterAlarmPin, (WaterTempAverage >= Settings[WATER])); 
  Alarms[FLOW] = (l_min * 60 <= Settings[FLOW]);
  digitalWrite( fluxAlarmPin, (l_min * 60 <= Settings[FLOW])); 

  // in case of alarm triggered, then open the relay and play sound
 if (Alarms[TUBE] || Alarms[WATER] ||Alarms[FLOW]) 
 {
  digitalWrite( relayPin, LOW);
  if (!snooze) tone(buzzerPin, 840, 100);
 }
 else
 {
  digitalWrite( relayPin, HIGH);
  snooze = LOW;
 }

  // Read CST buttons and adapt cursor position/set value accordingly
  ctsValueUP = digitalRead(ctsPinUP);
  ctsValueDOWN = digitalRead(ctsPinDOWN);

  // momorize old position to detect button toggle
  if (ctsValueUP == HIGH) ctsPreviousUP = HIGH;
  if (ctsValueDOWN == HIGH) ctsPreviousDOWN = HIGH;

  // toggle snooze sound ... thank you
  if (ctsValueUP == HIGH && ctsValueDOWN == HIGH)  snooze = !snooze;
   
  // if cursor position changed, then clear the display to avoid scrambled display
  if (cursor != oldcursor)   display.clear();
  oldcursor = cursor;

  // Read switch position : HIGH=set value, LOW=set cursor position
  if (swPosition != digitalRead(swPin)) display.clear();
  swPosition = digitalRead(swPin);

  // Mode READ then dislay the value under cursor and switch on the corresponding LED to show which value is displayed
  if (swPosition == LOW) 
  {
    // Update the cursor position
    if ((ctsValueUP == LOW) && (ctsPreviousUP == HIGH)) 
    {
      ctsPreviousUP = LOW;
      cursor = ( cursor + 2) % 3; // crusor up one position, rollup if needed
    }
    if ((ctsValueDOWN == LOW) && (ctsPreviousDOWN == HIGH)) 
    {
      ctsPreviousDOWN = LOW;
      cursor = ( cursor + 1) % 3; // cursor down one position, rollup if needed
    }

    // show value under cursor
    switch (cursor)
    {
      case FLOW:
        digitalWrite( fluxSelectPin, HIGH);
        digitalWrite( waterSelectPin, LOW);
        digitalWrite( tubeSelectPin, LOW);
        display.showNumberDec(l_min * 60, false, 3, 0);
        display.setSegments(STRflow, 1, 3);
        break;
      case WATER:
        digitalWrite( fluxSelectPin, LOW);
        digitalWrite( waterSelectPin, HIGH);
        digitalWrite( tubeSelectPin, LOW);
        display.showNumberDec(WaterTempAverage, false, 2, 0);
        display.setSegments(STRcelsius, 2, 2);
        break;
      case TUBE:
        digitalWrite( fluxSelectPin, LOW);
        digitalWrite( waterSelectPin, LOW);
        digitalWrite( tubeSelectPin, HIGH);
        display.showNumberDec(TubeTempAverage, false, 2, 0);
        display.setSegments(STRcelsius, 2, 2);
        break;
    }   
  }
  // Mode SET, set alarm values
  else 
  {
    if ((ctsValueUP == LOW) && (ctsPreviousUP == HIGH)) 
    {
      ctsPreviousUP= LOW;
      Settings[ cursor]++;
      if (Settings[ cursor]>200) Settings[ cursor]=200 ; // to avoid irrealistic values
      EEPROM.put(0, Settings); // save new settings
    }
    if ((ctsValueDOWN == LOW) && (ctsPreviousDOWN == HIGH)) 
    {
      ctsPreviousDOWN= LOW;
      Settings[ cursor]--;
      if (Settings[ cursor]<0) Settings[ cursor]=0 ;  // to avoid negative values
      EEPROM.put(0, Settings);  // save new settings
    }

    switch (cursor)
    {
      case FLOW:
        display.showNumberDec(Settings[ FLOW], false, 3 , 0); 
        display.setSegments(STRflow, 1, 3);
        break;
      case WATER:
        display.showNumberDec(Settings[ WATER], false, 2 , 0);
        display.setSegments(STRcelsius, 2, 2);  
        break;
      case TUBE:
        display.showNumberDec(Settings[ TUBE], false, 2 , 0);
        display.setSegments(STRcelsius, 2, 2);   
        break;
    }   
  }
 }
