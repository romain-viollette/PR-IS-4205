#define _DEBUG false

#include <SimpleTimer.h>   // http://arduino.cc/playground/Code/SimpleTimer
#include "include.h"
#include <EEPROM.h>
#include "eeprom.h"


/**
 * Asservissement d'un moteur à l'aide d'un régulateur PID
 * Avril 2012 - Black Templar
 * http://www.ferdinandpiette.com/blog/2012/04/asservissement-en-vitesse-dun-moteur-avec-arduino/
 */

/* Routine d'initialisation */
void setup() {

  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(encodPinA1, INPUT); 
  pinMode(encodPinB1, INPUT); 
  pinMode(encodPinA2, INPUT); 
  pinMode(encodPinB2, INPUT); 

  roueG.kp = eepromReadLong(EEPROM_L_KP);
  roueG.ki = eepromReadLong(EEPROM_L_KI);
  roueG.kd = eepromReadLong(EEPROM_L_KD);

  roueD.kp = eepromReadLong(EEPROM_R_KP);
  roueD.ki = eepromReadLong(EEPROM_R_KI);
  roueD.kd = eepromReadLong(EEPROM_R_KD);     

  Serial.begin(19200);         // Initialisation port COM

  attachInterrupt(0, compteurG, FALLING);
  attachInterrupt(1, compteurD, FALLING);
  timer.setInterval(1000/frequence_echantillonnage, processPIDs);  // Interruption pour calcul du PID et asservissement

  Serial.println("\nBonjour!");
}

/* Fonction principale */
void loop(){
  timer.run();
  //timer2.run();
  //delay(10);
  if (stringComplete)
  {
    // Parse the recieved data
    ParseSerialData();
    // Reset inString to empty
    inString = "";
    for(int i=0; i< COMMAND_LENGTH; i++)
    {
      inData[i]=0;
    } 
    // Reset the system for further
    // input of data
    stringComplete = false;
  }
}

/* Interruption sur tick de la codeuse */
void compteurG(){
  if (PIND & (1<<4))
    roueG.tick_codeuse--;
  else
    roueG.tick_codeuse++;
}

void compteurD(){
  if (PIND & (1<<5))
    roueD.tick_codeuse--;
  else
    roueD.tick_codeuse++;
}

void computeSpeedPolicy()
{
      const long inc = 3;
      static long sens = inc;
      unsigned long elapsedTime = millis();
      
      if(roueG.tick_codeuse > (roueG.consigne>>1))
        accelerationSign = ACCELERATION;
      else
        accelerationSign = DECELERATION;
      
      if(consig
      
      
      
      //Serial.println(roueD.consigne);

      roueG.regime = MARCHE;
      roueD.regime = MARCHE;
      
      if (roueG.consigne > 1000) sens = -inc;
      if (roueG.consigne < 2) sens = +inc;
      
      roueG.consigne = roueD.consigne += sens;
      
      
}

void processPIDs()
{ 
  computeSpeedPolicy();
  asservissement(&roueG);
  asservissement(&roueD);

  if(roueG.regime == ARRET)
  {
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, LOW);
    analogWrite(PWM1, 0);
    roueG.sortie = 0;
    roueG.consigne = roueG.somme_erreur = roueG.erreur_precedente = 0.0;
  }
  else
  {
    if(roueG.sortie > 0)
    {
      digitalWrite(InA1, HIGH);
      digitalWrite(InB1, LOW);
      analogWrite(PWM1, roueG.sortie);
    }
    else if(roueG.sortie == 0)
    {
      digitalWrite(InA1, LOW);
      digitalWrite(InB1, LOW);
      analogWrite(PWM1, 0);
    }
    else
    {
      digitalWrite(InA1, LOW);
      digitalWrite(InB1, HIGH);
      analogWrite(PWM1, -roueG.sortie);
    }
  }

  if(roueD.regime == ARRET)
  {
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, LOW);
    analogWrite(PWM2, 0);
    roueD.sortie = 0;
    roueD.consigne = roueD.somme_erreur = roueG.erreur_precedente = 0.0;
  }
  else
  {
    if(roueD.sortie > 0)
    {
      digitalWrite(InA2, HIGH);
      digitalWrite(InB2, LOW);
      analogWrite(PWM2, roueD.sortie);
    }
    else if(roueD.sortie == 0)
    {
      digitalWrite(InA2, LOW);
      digitalWrite(InB2, LOW);
      analogWrite(PWM2, 0);
    }
    else
    {
      digitalWrite(InA2, LOW);
      digitalWrite(InB2, HIGH);
      analogWrite(PWM2, -roueD.sortie);
    } 
  }
}

/* Interruption pour calcul du PID */
void asservissement(Roue *roue)
{
  // Réinitialisation du nombre de tick de la codeuse
  //int tick = roue->tick_codeuse;
  //roue->tick_codeuse=0;

  // Calcul des erreurs
  //int frequence_codeuse = frequence_echantillonnage * tick;
  
  //roue->nb_tour_par_sec = (float)frequence_codeuse/(float)tick_par_tour_codeuse/(float)rapport_reducteur;
  
  //float erreur = (float)roue->consigne - roue->nb_tour_par_sec;
 
  long erreur = roue->consigne - roue->tick_codeuse;
  roue->somme_erreur += erreur;
  long delta_erreur = erreur - roue->erreur_precedente;
  roue->erreur_precedente = erreur;

  byte const decalage = 10;
  // PID : calcul de la commande
  long cmd = roue->kp*erreur + roue->ki*roue->somme_erreur + roue->kd*delta_erreur;
  cmd = cmd >>decalage;
  // Normalisation et contrôle du moteur
  if(cmd < -255) cmd=-255;
  else if(cmd > 255) cmd = 255;

  roue->sortie = cmd; 

  // DEBUG
  if(_DEBUG)  
  {
    if(roue == &roueG)
      Serial.print("Droite: ");
    else if(roue == &roueD)
      Serial.print("Gauche: ");
      
    Serial.print("  consigne: ");
    Serial.print(roue->consigne);
    Serial.print("  cmd: ");
    Serial.print(cmd);
    //Serial.print("  frequence_codeuse: ");
    //Serial.print(frequence_codeuse);
    Serial.print("  erreur: ");
    Serial.print(erreur);
    Serial.print("  somme_erreur: ");
    Serial.print(roue->somme_erreur, 4);
    Serial.print("  delta_erreur: ");
    Serial.print(delta_erreur, 4);
    Serial.print("  erreur_precedente: ");
    Serial.print(roue->erreur_precedente);
    Serial.print("  cmd: ");
    Serial.print(cmd);
    Serial.println();
  }
}

void serialEvent()
{
  // Read while we have data
  while (Serial.available() && stringComplete == false)
  {
    if(index>=COMMAND_LENGTH)
    {
      Serial.flush();
      stringComplete =true;
      index = 0;
      return;
    }
    // Read a character
    char inChar = Serial.read();

    // Check for termination character
    if (inChar == '\n' || inChar == '\r')
    {
      // Reset the index
      index = 0;
      // Set completion of read to true
      stringComplete = true; 
      return;
    }

    // Store it in char array
    inData[index] = inChar;
    // Increment where to write next
    index++;
    // Also add it to string storage just
    // in case, not used yet!
    inString += inChar;
  }
}

void ParseSerialData()
{
  // The data to be parsed
  char *p = inData;
  // Temp store for each data chunk
  char *str;
  // Id ref for each chunk
  int count = 0;
  // Loop through the data and seperate it into
  // chunks at each " " delimeter
  while ((str = strtok_r(p, " ", &p)) != NULL)
  {
    // Add chunk to array
    inParse[count] = str;
    // Increment data count
    count++;
  }

  if(count == 1)
  {
    if(strcmp(inParse[0],"stop")==0)
    {
      Serial.print("Stopping ");
      roueG.regime = ARRET;
      roueD.regime = ARRET;
    }
    else
    {
      Serial.print("?? : ");
      Serial.print(inParse[0]);
    }
  }

  else if(count == 2)
  {
    if(strcmp(inParse[0],"print")==0)
    {
      if(strcmp(inParse[1],"pid")==0)
      { //"print pid"
        affichePID();
      }
      else if(strcmp(inParse[1],"debug")==0)
      { //"print debug"
        Serial.print("consigne: ");
        Serial.print(roueG.consigne);
        Serial.print(" | ");
        Serial.println(roueD.consigne);
        Serial.print("input (tick_codeuse): ");
        Serial.print(roueG.tick_codeuse);
        Serial.print(" | ");
        Serial.println(roueD.tick_codeuse);
        Serial.print("sortie: ");
        Serial.print(roueG.sortie);
        Serial.print(" | ");
        Serial.println(roueD.sortie);
      }
      else
      {
        Serial.print("?? : ");
        Serial.print(inParse[0]); 
        Serial.print(" ");
        Serial.println(inParse[1]); 
      }

    }
    else if(strcmp(inParse[0],"step")==0)
    { //"step 123"
      float cmd = atof(inParse[1]);

      Serial.print("consigne: ");
      Serial.print(roueG.consigne);
      Serial.print(" -> ");
      Serial.println(cmd);

      Serial.print(roueD.consigne);
      Serial.print(" -> ");
      Serial.println(cmd);

      roueG.regime = MARCHE;
      roueD.regime = MARCHE;

      roueG.consigne += cmd;
      roueD.consigne += cmd;
    }

    else
    {
      Serial.print("?? : ");
      Serial.print(inParse[0]); 
      Serial.print(" ");
      Serial.println(inParse[1]); 
    }

  }

  else if(count == 3)
  {
    if(strcmp(inParse[0],"pid")==0)
    {
      char cmd1 = inParse[1][0];    // 'p' ou 'i' ou 'd'
      float cmd2 = atof(inParse[2]);     //valeur 12.345 

      if(cmd1 == 'p')
      { //pid p 5"
        eepromUpdateLong(EEPROM_L_KP,cmd2);
        roueG.kp = cmd2;
        eepromUpdateLong(EEPROM_R_KP,cmd2);
        roueD.kp = cmd2;
      }
      else if(cmd1 == 'i')
      {  //pid i 5"
        eepromUpdateLong(EEPROM_L_KI,cmd2);
        roueG.ki = cmd2;
        eepromUpdateLong(EEPROM_R_KI,cmd2);
        roueD.ki = cmd2;
      }
      else if(cmd1 == 'd')
      { //pid d 5"
        eepromUpdateLong(EEPROM_L_KD,cmd2);
        roueG.kd = cmd2;
        eepromUpdateLong(EEPROM_R_KD,cmd2);
        roueD.kd = cmd2;
      }
      affichePID();
    }
    else
    {
      Serial.print("?? : ");
      Serial.print(inParse[0]); 
      Serial.print(" ");
      Serial.print(inParse[1]); 
      Serial.print(" ");
      Serial.println(inParse[2]); 
    }
  }
  else if(count == 4)
  {
    if(strcmp(inParse[0],"pid")==0)
    {
      char cmd1 = inParse[1][0];     //'l' ou 'r'
      char cmd2 = inParse[2][0];     // 'p' ou 'i' ou 'd'
      float cmd3 = atof(inParse[3]);  //valeur 12.345

      Serial.println(cmd3);


      if(cmd1 == 'l')
      {
        if(cmd2 == 'p')
        { //pid l p 5"
          eepromUpdateLong(EEPROM_L_KP,cmd3);
          roueG.kp = cmd3;
        }
        else if(cmd2 == 'i')
        {  //pid l i 5"
          eepromUpdateLong(EEPROM_L_KI,cmd3);
          roueG.ki = cmd3;
        }
        else if(cmd2 == 'd')
        { //pid l d 5"
          eepromUpdateLong(EEPROM_L_KD,cmd3);
          roueG.kd = cmd3;
        }
      }
      else if(cmd1 == 'r')
      {
        if(cmd2 == 'p')
        { //pid r p 5"
          eepromUpdateLong(EEPROM_R_KP,cmd3);
          roueD.kp = cmd3;
        }
        else if(cmd2 == 'i')
        { //pid r i 5"
          eepromUpdateLong(EEPROM_R_KI,cmd3);
          roueD.ki = cmd3;
        }
        else if(cmd2 == 'd')
        { //pid r d 5"
          eepromUpdateLong(EEPROM_R_KD,cmd3);
          roueD.kd = cmd3;
        }
      }
      affichePID();
    }
    else
    {
      Serial.print("?? : ");
      Serial.print(inParse[0]); 
      Serial.print(" ");
      Serial.print(inParse[1]); 
      Serial.print(" ");
      Serial.print(inParse[2]); 
      Serial.print(" ");
      Serial.println(inParse[3]); 
    }
  }
}


void affichePID()
{
  Serial.print("L: ");
  Serial.print("p = ");
  Serial.print(roueG.kp);
  Serial.print("   i = ");
  Serial.print(roueG.ki);
  Serial.print("   d = ");
  Serial.println(roueG.kd); 

  Serial.print("R: ");
  Serial.print("p = ");
  Serial.print(roueD.kp);
  Serial.print("   i = ");
  Serial.print(roueD.ki);
  Serial.print("   d = ");
  Serial.println(roueD.kd); 
}









