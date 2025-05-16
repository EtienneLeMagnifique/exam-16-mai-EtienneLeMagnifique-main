//**************************************************************************************************
// Fichier: main.cpp
// Auteur: Maxime Champagne
// Date: 2024/05/29
// Description: Programme pour contrôler un servo moteur PARALLAX continuous rotation en se basant
//              sur la mesure d'un magnétomètre CMPS12. L'angle de consigne est ajusté par le port
//              série.
//
//              BROCHE 12 utilisée pour envoyer un signal PWM pour contrôler le servo moteur.
//              BROCHES SCL et SDA sont utilisées pour communiquer avec le magnétomètre.
//
//**************************************************************************************************
#include <Arduino.h>
#include <Wire.h>

//Déclaration des fonctions
void lireCompas(void);
void algorithmePID(void);
void lecturePortSerie(void);
void afficheDegbug(void);

//Variables globales et defines pour timer0
#define INTERVAL_INTERRUPT 50000 //interval en us, 50000 -> 50ms
bool bPrintSignal = false;
hw_timer_t *Timer0_Cfg = NULL;

//Variables globales et defines pour algorithme
#define KP 20

float angleConsigne = 120;
float angleActuelle;
float erreur = 0; //erreur
int16_t sortieDuControleur = 0;

//Variables globales et defines pour controle LEDC
#define GPIO_CTRL_MOTOR 12  // 12 Correspond au GPIO 12
#define FREQ 50
#define LED_CHANNEL 0
#define RESOLUTION 16

//*************************************   Explication LEDC   ***************************************
//
//Explication fonctionnement ledc:
// LEDC est un module de PWM matériel qui permet de générer des signaux PWM sur n'importe quelle
// broche GPIO. 
// La fréquence du PWM est configurée en appelant ledcSetup(). 
// La résolution du PWM est configurée en appelant ledcSetup().
// Le duty cycle du PWM est configuré en appelant ledcWrite().
//
// Dans le code ci-dessous on configure la fréquence à 50Hz, donc période de 20ms.
// La résolution est de 16 bits, donc duty cycle de 0 à 65535.
// L'offset de départ est de 4900, donc duty cycle de 7,47% puisque 4900 / 65535 = 0,0747.
// Le temps haut du PWM est donc de 1,49ms (20ms * 0,0747) et le temps bas de 18,51ms (20ms - 1,49ms)
//**************************************************************************************************

//Variables globales pour contrôle moteur
uint16_t gOffset = 4900;
uint16_t ctrlMoteur = 0;


//define pour le magnétomètre CMPS12  
#define CMPS12_I2C_ADDR 0x60 //0xC0 >> 1bit puisque le bit 0 est pour write


//Routine d'interruption du timer0
void IRAM_ATTR Timer0_ISR()
{
  bPrintSignal = true;
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}


void setup()
{
  /* Initialisation de la led de la carte en sortie (GPIO) */
  pinMode(LED_BUILTIN, OUTPUT); //Attention, LED_BUILTIN aussi relié à la broche 13.

  /* initialisation de la communication serie avec l'ordinateur */
  Serial.begin(115200);
  Serial.println("<Port série ok>");

  // configure fonctionnalité PWM
  ledcSetup(LED_CHANNEL, FREQ, RESOLUTION);
  ledcAttachPin(GPIO_CTRL_MOTOR, LED_CHANNEL);

/* Initialisation de la composante de communication I2C */
  Wire.begin();        /* initialisation i2c pour l'ESP32 */
  delay(1000);         // Attendre que le CMPS12 soit prêt

  //Configuration du timer0
  Timer0_Cfg = timerBegin(0, 80, true);                   //Confifure timer 0 à 1 tick par us et activer réactivation automatique
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, INTERVAL_INTERRUPT, true);  //Interruption quand le timer atteint INTERVAL_INTERRUPT
  timerAlarmEnable(Timer0_Cfg);

}




void loop()
{
  /* Lecture de l'angle actuel en degré avec le cmps12 */
  lireCompas();

  /* Calcul de la commande PID a envoyer au servo  */
  algorithmePID();

  /* Application de l'offset (decalage) pour centrer le moteur */
  ctrlMoteur = sortieDuControleur + gOffset;

  //Limite les valeurs de ctrlMoteur
  //Limite à minimum 0,9ms (20ms/65535*0.9ms)
  //Limite à maximum 2.1ms (20ms/65535*2.1ms)
  if (ctrlMoteur < 2949){ ctrlMoteur = 2949;}
  else if (ctrlMoteur > 6881){ ctrlMoteur = 6881;}  

  /* Envoi du signal au servomoteur en PWM */
  ledcWrite(LED_CHANNEL, ctrlMoteur);

  /* Lecture des commandes envoyer par le port serie pour ajuster l'angle ou le offset */
  lecturePortSerie();

  /* Affiche les informations pour faire le debugage */
  afficheDegbug();

}

void lireCompas(void){
  uint8_t angle_0_255 = 0;

  Wire.beginTransmission(CMPS12_I2C_ADDR);            /* Début de la transmission i2c */
  Wire.write(0x01);                                   /* ecriture du caractère 0x01 (registre pour l'angle) */
  Wire.endTransmission(false);                        /* Arrete de la transmission i2c */

  Wire.requestFrom(CMPS12_I2C_ADDR, 1, (int)true);    /* demande de lire le 1er octet dans le cmps12 */
  angle_0_255 = Wire.read();                          /* Lecture de l'angle (fourni par le cmps12) */
  
  angleActuelle = angle_0_255 * 360.0 / 255.0;        /* Conversion en degré */
}

void algorithmePID(void){
  
  erreur =  (angleActuelle - angleConsigne);  /* création de l'erreur */

  //Ajustement de l'erreur pour éviter les problèmes lorsque l'angleActuelle est proche de 0 ou 360
  //Ce code est fonctionnel.
  if (erreur <= -180) {
      erreur = erreur + 360;
  }
  else if (erreur > 180) {
      erreur = erreur - 360;
  }

  //TODO: Ajouter code errreur intégrale



  //TODO: Ajouter code errreur dérivée




  
  sortieDuControleur = (KP * erreur);         /* Calcul de la sortie du controleur */
}


/* Lit le caractere dans le port série */
void lecturePortSerie(void){
  byte incomingByte;

  if (Serial.available()){
    /* quand on écrit un caractère sur le port serie */

    incomingByte = Serial.read(); // Lire le caractère reçu
    if('1' == incomingByte){        /* Si le caractère lu est 1 */
      angleConsigne = 120;          /* mettre la consigne à 120 degres */
    }
    else if ('2' == incomingByte){  /* Si le caractère lu est 2 */
      angleConsigne = 240;          /* Mettre la consigne à 240 degres */
    }
    else if ('p' == incomingByte){  /* Si le caractère lu est p */
      gOffset += 1;                 /* ajuster le offset du moteur de +1 */
    }
    else if ('P' == incomingByte){  /* Si le caractère lu est P */
      gOffset += 10;                /* ajuster le offfset du moteur de +10 */
    }    
    else if ('l' == incomingByte){  /* Si le caractère lu est l */
      gOffset -= 1;                 /* ajuster le offfset du moteur de -1 */
    }  
    else if ('L' == incomingByte){  /* Si le caractère lu est L */
      gOffset -= 10;                /* ajuster le offfset du moteur de -10 */
    }            
  }
}

/* Affiche un message sur le port serie pour pouvoir debuger */
void afficheDegbug(void){

  if (true == bPrintSignal){    /* Si l'interruption dit de faire cette commande */

    //Serial.print("Message de debug\n\r");

    // Affiche l'angle mesuré et la commande PWM sous forme de valeurs séparées par une virgule.
    Serial.print(angleActuelle);
    Serial.print(",");
    Serial.println(ctrlMoteur);
    
    
    bPrintSignal = false;
  }
}