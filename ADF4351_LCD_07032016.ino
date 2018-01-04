//   ADF4251 and Arduino
//   By Alain Fort F1CJN feb 2,2016
//   update march 7, 2016 (ROBOT V1.1 and V1.0)
//
//
//  ****************************************************** FRANCAIS ****************************************************
//   Ce programme utilise un Arduino Uno muni d'un "LCD button shield" de marque ROBOT, avec boutons permettant de commander
//   un ADF4351 qui generer une frequence comprise entre 34,5 et 4400MHz.
//   Vingt fréquences peuvent être memorisees dans le memoire EEPROM de l'Arduino.
//   Si une ou plusieurs fréquence sont mémorisées, alors la fréquence en memoire zero sera affichee à la mise sous tension
//   
//   Le curseur se deplace avec les touches LEFT and RIGHT. Les digits placés sous le curseur peuvent être modifiées avec
//   les touches UP et DOWN, ceci pour la fréquence, la memoire et la frequence de reference:
//   - pour la fréquence, il suffit de placer le curseur sur le chiffre à modifier,
//   - pour la mémoire , il suffit de placer le curseur sur le numero de memoire,
//   - pour la fréqence de référence, il suffit de placer le curseur sur 10 ou 25,
//   - pour la lecture ou écriture de la frequence en memoire, placer le curseur en bas à gauche (passage de REE(lecture
//    EEPROM) à WEE(Ecriture EEPROM).
//    Le curseur disparait apres quelques secondes et est re active lors de l'appui sur une touche.
//
//   MEMORISATION 
//   - pour la frequennce, mettre à WEE, puis selectionner le numero de memoire, puis appuyer sur la touche SELECT pendant 
//    une seconde. Le mot MEMORISATION apparait alors sur l'ecran. Ceci fonctionne quelquesoit le placement du curseur excepte sur 
//    l'emplacement de la fréquence de réference 10 ou 25.
//   - Pour la frequence de reference, placer le curseur sur 10 ou 25, puis appuyer pendant 1s sur la touche SELECT.
//
//  ********************************************* HARDWARE IMPORTANT *******************************************************
//  Avec un Arduino UN0 : utilise un pont de résistances pour réduire la tension, MOSI (pin 11) vers
//  ADF DATA, SCK (pin13) vers CLK ADF, Select (PIN 3) vers LE
//  Resistances de 560 Ohm avec 1000 Ohm à la masse sur les pins 11, 13 et 3 de l'Arduino UNO pour
//  que les signaux envoyés DATA, CLK et LE vers l'ADF4351 ne depassent pas 3,3 Volt.
//  Pin 2 de l'Arduino (pour la detection de lock) connectee directement à la sortie MUXOUT de la carte ADF4351
//  La carte ADF est alimentée en 5V par la carte Arduino (les pins +5V et GND sont proches de la LED Arduino).
//  ***********************************************************************************************************************
//  Attention : si vous utiliser un afficheur ROBOT Version 1.1 il faut modifier la routine de lecture des boutons
//  en enlevant les commantaires de la version 1.1 et en mettant en commentaires la version 1.0
//
//  *************************************************** ENGLISH ***********************************************************
//
//  This sketch uses and Arduino Uno (5€), a standard "LCD buttons shield" from ROBOT (5€), with buttons and an ADF4351 chineese
//  card found at EBAY (40€). The frequency can be programmed between 34.5 and 4400 MHz.
//  Twenty frequencies can be memorized into the Arduino EEPROM.
//  If one or more frequencies are memorized, then at power on, the memory zero is always selected.
//
//   The cursor can move with le LEFT and RIGHT buttons. Then the underlined digit can be modified with the UP and DOWN buttons, 
//    for the frequency, the memories and the frequency reference (10 or 25 MHz):
//   - to change the frequency, move the cursor to the digit to be modified, then use the UP and DOWN buttons,
//   - to modify the memory number,move the cursor to the number to be modified, then use the UP and DOWN buttons,
//   - to select the refrence frequence,move the cursor on 10 or 25 and select with UP and DOWN.
//   - to read or write the frequency in memory, place the cursor on the more left/more down position and select REE (for Reading EEprom)
//    or WEE (for Writing EEprom).
//    The cursor dissapears after few seconds and is re activated if a button is pressed.
//
//   MEMORIZATION 
//    - For the frequency, select WEE, then select the memory number, then push the SELECT button for a second. The word MEMORISATION 
//    appears on the screen. This memorization works then the cursor is anywhere except on the reference 10 or 25 position.
//    - For the reference frequency, move the cursor to 10 or 25, the press the SELECT for one second. 

//  ******************************************** HARDWARE IMPORTANT********************************************************
//  With an Arduino UN0 : uses a resistive divider to reduce the voltage, MOSI (pin 11) to
//  ADF DATA, SCK (pin13) to ADF CLK, Select (PIN 3) to ADF LE
//  Resistive divider 560 Ohm with 1000 Ohm to ground on Arduino pins 11, 13 et 3 to adapt from 5V
//  to 3.3V the digital signals DATA, CLK and LE send by the Arduino.
//  Arduino pin 2 (for lock detection) directly connected to ADF4351 card MUXOUT.
//  The ADF card is 5V powered by the ARDUINO (PINs +5V and GND are closed to the Arduino LED).

//************************************************* MANUEL*****************************************************************
//Touche LEFT    curseur à gauche, cursor to the left
//Touche RIGHT   curseur à droite, cursor to the right
//Touche UP      incremente frequence ou memoire, increase frequency
//Touche DOWN    decremente frequence ou memoire, decrease frequency
//Touche SELECT  long push = frequency memorization into the EE number EEPROM / or reference memorization
//*************************************************************************************************************************
// Warning : if you are using a ROBOT Shied version 1.1, it is necessary to modify the read_lcd_buttons sub routine 
// you need not to comment the 1.1 version and to comment the 1.0 version. See below

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SPI.h>

#define ADF4351_LE 3

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

byte poscursor = 0; //position curseur courante 0 à 15
byte line = 0; // ligne afficheur LCD en cours 0 ou 1
byte memoire,RWtemp; // numero de la memoire EEPROM

uint32_t registers[6] =  {0x4580A8, 0x80080C9, 0x4E42, 0x4B3, 0xBC803C, 0x580005} ; // 437 MHz avec ref à 25 MHz
//uint32_t registers[6] =  {0, 0, 0, 0, 0xBC803C, 0x580005} ; // 437 MHz avec ref à 25 MHz
int address,modif=0,WEE=0;
int lcd_key = 0;
int adc_key_in  = 0;
int timer = 0,timer2=0; // utilisé pour mesurer la durée d'appui sur une touche
unsigned int i = 0;


double RFout, REFin, INT, PFDRFout, OutputChannelSpacing, FRACF;
double RFoutMin = 35, RFoutMax = 4400, REFinMax = 250, PDFMax = 32;
unsigned int long RFint,RFintold,INTA,RFcalc,PDRFout, MOD, FRAC;
byte OutputDivider;byte lock=2;
unsigned int long reg0, reg1;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//**************************** SP LECTURE BOUTONS ********************************************
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the buttons
  if (adc_key_in < 790)lcd.blink();
  
  if (adc_key_in < 50)return btnRIGHT;  // pour Afficheur ROBOT V1.0
  if (adc_key_in < 195)return btnUP;
  if (adc_key_in < 380)return btnDOWN;
  if (adc_key_in < 555)return btnLEFT;
  if (adc_key_in < 790)return btnSELECT; // Fin Afficheur ROBOT1.1

  //if (adc_key_in < 50)return btnRIGHT; // pour Afficheur ROBOT 1.1
  //if (adc_key_in < 250)return btnUP;
  //if (adc_key_in < 450)return btnDOWN;
  //if (adc_key_in < 650)return btnLEFT;
  //if (adc_key_in < 850)return btnSELECT; // fin Afficheur ROBOT 1.1

  return btnNONE;  // touches non appuyees
}

//***************************** SP Affichage Fréquence sur LCD ********************************
void printAll ()
{
  //RFout=1001.10 // test
  lcd.setCursor(0, 0);
  lcd.print("RF = ");
  if (RFint < 100000) lcd.print(" ");
  if (RFint < 10000)  lcd.print(" ");
  lcd.print(RFint/100);lcd.print(".");
  RFcalc=RFint-((RFint/100)*100);
  if (RFcalc<10)lcd.print("0");
  lcd.print(RFcalc);
  lcd.print(" MHz");
  lcd.setCursor(0,1);
  if (WEE==0) {lcd.print("REE=");}
  else {lcd.print("WEE=");}
  if (memoire<10)lcd.print(" ");
  lcd.print(memoire,DEC);
  if  ((digitalRead(2)==1))lcd.print(" LOCKED ");
  else lcd.print(" NOLOCK ");
  lcd.print(PFDRFout,DEC);
  lcd.setCursor(poscursor,line);
}

void WriteRegister32(const uint32_t value)   //Programme un registre 32bits
{
  digitalWrite(ADF4351_LE, LOW);
  for (int i = 3; i >= 0; i--)          // boucle sur 4 x 8bits
  SPI.transfer((value >> 8 * i) & 0xFF); // décalage, masquage de l'octet et envoi via SPI
  digitalWrite(ADF4351_LE, HIGH);
  digitalWrite(ADF4351_LE, LOW);
}

void SetADF4351()  // Programme tous les registres de l'ADF4351
{ for (int i = 5; i >= 0; i--)  // programmation ADF4351 en commencant par R5
    WriteRegister32(registers[i]);
}

// *************** SP ecriture Mot long (32bits) en EEPROM  entre adress et adress+3 **************
void EEPROMWritelong(int address, long value)
      {
      //Decomposition du long (32bits) en 4 bytes
      //trois = MSB -> quatre = lsb
      byte quatre = (value & 0xFF);
      byte trois = ((value >> 8) & 0xFF);
      byte deux = ((value >> 16) & 0xFF);
      byte un = ((value >> 24) & 0xFF);

      //Ecrit 4 bytes dans la memoire EEPROM
      EEPROM.write(address, quatre);
      EEPROM.write(address + 1, trois);
      EEPROM.write(address + 2, deux);
      EEPROM.write(address + 3, un);
      }

// *************** SP lecture Mot long (32bits) en EEPROM situe entre adress et adress+3 **************
long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long quatre = EEPROM.read(address);
      long trois = EEPROM.read(address + 1);
      long deux = EEPROM.read(address + 2);
      long un = EEPROM.read(address + 3);

      //Retourne le long(32bits) en utilisant le shift de 0, 8, 16 et 24 bits et des masques
      return ((quatre << 0) & 0xFF) + ((trois << 8) & 0xFFFF) + ((deux << 16) & 0xFFFFFF) + ((un << 24) & 0xFFFFFFFF);
      }
//************************************ Setup ****************************************
void setup() {
  lcd.begin(16, 2); // two 16 characters lines
  lcd.display();
  analogWrite(10,255); //Luminosite LCD

  Serial.begin (19200); //  Serial to the PC via Arduino "Serial Monitor"  at 9600
  lcd.print("   GENERATEUR   ");
  lcd.setCursor(0, 1);
  lcd.print("    ADF4351     ");
  poscursor = 7; line = 0; 
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("   par F1CJN    ");
   delay(1000);

  pinMode(2, INPUT);  // PIN 2 en entree pour lock
  pinMode(ADF4351_LE, OUTPUT);          // Setup pins
  digitalWrite(ADF4351_LE, HIGH);
  SPI.begin();                          // Init SPI bus
  SPI.setDataMode(SPI_MODE0);           // CPHA = 0 et Clock positive
  SPI.setBitOrder(MSBFIRST);            // poids forts en tête

  if (EEPROM.read(100)==55){PFDRFout=EEPROM.read(20*4);} // si la ref est ecrite en EEPROM, on la lit
  else {PFDRFout=25;}

  if (EEPROM.read(101)==55){RFint=EEPROMReadlong(memoire*4);} // si une frequence est ecrite en EEPROM on la lit
  else {RFint=7000;}

  RFintold=1234;//pour que RFintold soit different de RFout lors de l'init
  RFout = RFint/100 ; // fréquence de sortie
  OutputChannelSpacing = 0.01; // Pas de fréquence = 10kHz

  WEE=0;  address=0;
  lcd.blink();
  printAll(); delay(500);


} // Fin setup

//*************************************Loop***********************************
void loop()
{
  RFout=RFint;
  RFout=RFout/100;
  if ((RFint != RFintold)|| (modif==1)) {
    //Serial.print(RFout,DEC);Serial.print("\r\n");
    if (RFout >= 2200) {
      OutputDivider = 1;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 2200) {
      OutputDivider = 2;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 1100) {
      OutputDivider = 4;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 550)  {
      OutputDivider = 8;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 275)  {
      OutputDivider = 16;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 137.5) {
      OutputDivider = 32;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 68.75) {
      OutputDivider = 64;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    }

    INTA = (RFout * OutputDivider) / PFDRFout;
    MOD = (PFDRFout / OutputChannelSpacing);
    FRACF = (((RFout * OutputDivider) / PFDRFout) - INTA) * MOD;
    FRAC = round(FRACF); // On arrondit le résultat

    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    FRAC = FRAC << 3;
    registers[0] = registers[0] + FRAC;

    registers[1] = 0;
    registers[1] = MOD << 3;
    registers[1] = registers[1] + 1 ; // ajout de l'adresse "001"
    bitSet (registers[1], 27); // Prescaler sur 8/9

    bitSet (registers[2], 28); // Digital lock == "110" sur b28 b27 b26
    bitSet (registers[2], 27); // digital lock 
    bitClear (registers[2], 26); // digital lock
   
    SetADF4351();  // Programme tous les registres de l'ADF4351
    RFintold=RFint;modif=0;
    printAll();  // Affichage LCD
  }

  lcd_key = read_LCD_buttons();  // read the buttons

  switch (lcd_key)               // Select action
  {
    case btnRIGHT: //Droit
      poscursor++; // cursor to the right
      if (line == 0) {
        if (poscursor == 9 ) {
          poscursor = 10;
          line = 0; } //si curseur sur le .
        if (poscursor == 12 ) {
          poscursor = 0; line = 1; }; //si curseur à droite
      }
     if (line == 1) {
        if (poscursor == 1 ) {poscursor = 5; line = 1; } //si curseur sur le chiffre memoire 
        if (poscursor == 6 ) {poscursor = 15; line = 1; } //si curseur sur le chiffre memoire 
        if (poscursor==16) {poscursor=5; line=0;};     
      }  
      //Serial.print (" RIGHT Button\r\n");
      lcd.setCursor(poscursor, line);
      break;
      
    case btnLEFT: //Gauche
      poscursor--; // décalage curseur
      if (line == 0) {
        if (poscursor == 4) {poscursor = 15; line = 1;  };
        if (poscursor == 9) {   poscursor = 8; line=0;}
      }
       if(line==1){
          if (poscursor==255) {poscursor=11; line=0;};
          if (poscursor==4) {poscursor=0; line=1;};
          if (poscursor==14) {poscursor=5; line=1;};
      }
      //Serial.print(poscursor,DEC);  
      lcd.setCursor(poscursor, line);
      break;
      
    case btnUP: //Haut
      if (line == 0)
      { // RFoutfrequency
        //Serial.print(oldRFint,DEC);
        if (poscursor == 5) RFint = RFint + 100000 ;
        if (poscursor == 6) RFint = RFint + 10000 ;
        if (poscursor == 7) RFint = RFint + 1000 ;
        if (poscursor == 8) RFint = RFint + 100 ;
        if (poscursor == 10) RFint = RFint + 10 ;
        if (poscursor == 11) RFint = RFint + 1 ;
        if (RFint > 440000)RFint = RFintold;
        //Serial.print(RFint,DEC);
        //Serial.print("  \r\n");
      }
      if (line == 1)
      { 
        if (poscursor == 5){ memoire++; 
        if (memoire==20)memoire=0;
        if (WEE==0){RFint=EEPROMReadlong(memoire*4); // lecture EEPROM et Affichage
           if (RFint>440000) RFint=440000; 
           } 
        }  
        if (poscursor==15){ 
        if( PFDRFout==10){PFDRFout=25;} //reglage FREF
        else if ( PFDRFout==25){PFDRFout=10;}
        else PFDRFout=25;// au cas ou PFDRF different de 10 et 25
        modif=1;  }
                    
      if( (poscursor==0) && (WEE==1))WEE=0;
      else if ((poscursor==0) && (WEE==0))WEE=1;                  
      }
        printAll();
      break; // fin bouton up

    case btnDOWN: //bas
      if (line == 0) {
        if (poscursor == 5) RFint = RFint - 100000 ;
        if (poscursor == 6) RFint = RFint - 10000 ;
        if (poscursor == 7) RFint = RFint - 1000 ;
        if (poscursor == 8) RFint = RFint - 100 ;
        if (poscursor == 10) RFint = RFint - 10 ;
        if (poscursor == 11) RFint = RFint - 1 ;
        if (RFint < 3450) RFint = RFintold;
        if (RFint > 440000)  RFint = RFintold;
        break;
      }

     if (line == 1)
      { 
        if (poscursor == 5){memoire--; 
        if (memoire==255)memoire=19;
        if (WEE==0){RFint=EEPROMReadlong(memoire*4); // lecture EEPROM et Affichage
           if (RFint>440000) RFint=440000;
          // Serial.print(RFint,DEC);  
           } 
        } // fin poscursor =5 

       if (poscursor==15){ 
       if( PFDRFout==10){PFDRFout=25;} //reglage FREF
       else if ( PFDRFout==25){PFDRFout=10;}
       else PFDRFout=25;// au cas ou PFDRF different de 10 et 25
       modif=1;
       }
                   
       if( (poscursor==0) && (WEE==1))WEE=0;
       else if ((poscursor==0)&&(WEE==0))WEE=1;                          
      
       printAll();
      // Serial.print (" DOWN Button  \r\n");
      break; // fin bouton bas
      }

    case btnSELECT:
      do {
        adc_key_in = analogRead(0);      // Test release button
        delay(1); timer2++;        // timer inc toutes les 1 millisecondes
        if (timer2 > 600) { //attente 600 millisecondes
         if (WEE==1 || poscursor==15){ 
         if (line==1 && poscursor==15){ EEPROMWritelong(20*4,PFDRFout);EEPROM.write(100,55);} // ecriture FREF
         else if (WEE==1) {EEPROMWritelong(memoire*4,RFint);EEPROM.write(101,55);}// ecriture RF en EEPROM à adresse (memoire*4)
          lcd.setCursor(0,1); lcd.print("  MEMORISATION  ");}
          lcd.setCursor(poscursor,line);
          delay(500);timer2=0;
          printAll();
        }; // mes

        } 
      while (adc_key_in < 900); // attente relachement
      break;  // Fin bouton Select

     case btnNONE: {
        break;
      };
      break;
  }// Fin LCD keys

   do { adc_key_in = analogRead(0); delay(1);} while (adc_key_in < 900); // attente relachement touche
   delay (10);timer++; // inc timer
   //Serial.print(timer,DEC);
   if (timer>1000){lcd.noBlink();timer=0;} // curseur off

}   // fin loop


