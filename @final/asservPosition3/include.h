#define InA1            8                      // INA motor pin
#define InB1            10                      // INB motor pin 
#define PWM1            9                       // PWM motor pin
#define encodPinA1      2                       // encoder A pin
#define encodPinB1      4                       // encoder B pin

#define InA2            13                      // INA motor pin
#define InB2            12                      // INB motor pin 
#define PWM2            11                       // PWM motor pin
#define encodPinA2      3                       // encoder A pin
#define encodPinB2      5                       // encoder B pin

#define ARRET           0
#define MARCHE          1
#define ROUELIBRE       2

// Max command LENGTH
#define COMMAND_LENGTH  100
// Buffer for the incoming data
char inData[COMMAND_LENGTH];
// Buffer for the parsed data chunks
char *inParse[COMMAND_LENGTH];
// Storage for data as string
String inString = "";
// Incoming data id
int index = 0;
// Read state of incoming datac
boolean stringComplete = false;



const byte incrementMax = 10;
const byte ARRET_ =             0;
const byte ACCELERATION_ =      1;
const byte CROISIERE_ =         2;
const byte DECELERATION_ =      3;



typedef struct AsrVit{
  long alpha, increment;
  byte etat;
};

typedef struct Roue{
  long tick_codeuse, tick_parcouru, consigne;
  long sortie, consigne_finale, consigne_moteur, consigne_precedante ;
  long kp, ki, kd;
  long somme_erreur, erreur_precedente;
  long regime;
  AsrVit asrVit;
};

Roue roueG, roueD = {
  0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 
  0, 0,
  ARRET,
  {
   0, 0,
   ARRET_
  }
};

void asservissement(Roue*);

SimpleTimer timer;                 // Timer pour échantillonnage

const int frequence_echantillonnage = 20;  // Fréquence du pid
const int frequence_callback = 2;  // Fréquence du pid
const int tick_par_tour_codeuse = 3;
const int rapport_reducteur = 30;          // Rapport entre le nombre de tours de l'arbre moteur et de la roue

