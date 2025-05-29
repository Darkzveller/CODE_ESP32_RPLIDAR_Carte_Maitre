#include <Arduino.h>
#include <RPLidar.h>
#include <CAN/src/CAN.h>
#include "ident_crac.h"
#include "HuskyLens/HUSKYLENS.h"
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define RPLIDAR_MOTOR 18 // Pin du moteur du Lidar
#define PWM_CHANNEL 0    // Canal PWM pour le contrôle du moteur
#define RECTANGLE_X 3000.0f  // Largeur du terrain en mm
#define RECTANGLE_Y 2000.0f  // Hauteur du terrain en mm
#define DISTANCE_LIMITE 10.0f // Marge de sécurité pour les bords du terrain

#define MAX_ROBOTS 3        // Nombre maximal de robots à détecter
#define MOVING_AVG_WINDOW 5 // Taille de la fenêtre pour la moyenne mobile

// Structure pour stocker les informations sur les robots détectés
struct RobotInfo
{
  int id;                             // Identifiant du robot (de 1 à 3)
  float x;                            // Position en X du robot
  float y;                            // Position en Y du robot
  bool found;                         // Indique si le robot a été trouvé lors de la dernière itération
  int detection_counter;              // Compteur pour estimer la fiabilité de la détection
  float x_history[MOVING_AVG_WINDOW]; // Historique des positions en X pour la moyenne mobile
  float y_history[MOVING_AVG_WINDOW]; // Historique des positions en Y pour la moyenne mobile
  int history_index;                  // Index courant dans les historiques de positions
};
RobotInfo robots[MAX_ROBOTS]; // Tableau pour stocker les informations sur les robots détectés

RPLidar lidar;

// Valeur du robot envoyé par le CAN
float x_robot;
float y_robot;
float theta_robot;

bool newObject = false;

//CAN-----------------------------------------------
typedef struct CANMessage {
  bool extented = false;
  bool RTR = false;
  unsigned int ID;
  char ln;
  unsigned char dt[8];
} CANMessage;
bool CANAvailable = false;

CANMessage rxMsg;
CANMessage txMsg;

bool objetDetect = false;
float angle_debut = 0, angle_fin = 0;


//Huskylens-----------------------------------------
HUSKYLENS huskylens;

float ErreurPos = 0, last_ErreurPos = 0, Somme_ErreurPos = 0;
float Kpp = 0.0000004, Kip = 0.0002, Kdp =0.00005;
unsigned long wait = 0;

float currentPos = 0.0;


short Odo_x=0, Odo_y=0, Odo_theta=0;

bool asservissementEnable = true;
//----------------------------------------------------------------------Timer

int totalInterrupts = 0;   // compte le nombre de declenchement de l alarme
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
uint16_t          mscount/*temps d'echantionnage*/ = 0, mscount2/*bus can*/ = 0;
void init_Timer();
void onTime();//prototype de la fonction s'exécutent à chaque interruptions
//----------------------------------------------------------------------prototypes fonctions
//CAN
void setupCAN();
void CANloop();
void canReadData(int packetSize);
void canReadExtRtr();
void writeStructInCAN(const CANMessage &theDATA);
void printStruct(CANMessage msg);
void remplirStruct(CANMessage &theDATA, int idf, char lenf, 
                  char dt0f, char dt1f, char dt2f, char dt3f, 
                  char dt4f, char dt5f, char dt6f, char dt7f);

//Lidar
void setupLidar();
void initRPLidarA3M1(int frequence, int resolution, int vitesse);
void detectionRobots(float x_robot, float y_robot, float theta_robot);
void addOrUpdateRobot(float x, float y);
void envoiCAN();


void initRPLidarA3M1(int frequence, int resolution, int vitesse);
void detectionRobots(float x_robot, float y_robot, float theta_robot);
void addOrUpdateRobot(float x, float y);

void uartTask(void *pvParameters)
{
  while(1) {
    

    CANloop();

  // detectionRobots(x_robot, y_robot, theta_robot);

  // mscount2++;
  // if(mscount2 > 40*25*3){//300ms Pour ne pas surcharger le bus CAN
  //   mscount2 = 0;
  //   huskylensLoop();
  //   envoiCAN();
  // }
  
  // // Serial print pour débugger
  // if(newObject){
  //   newObject = false;
  //   Serial.printf("Robot %d: x = %4.1f, y = %4.1f    Robot %d: x = %4.1f, y = %4.1f     Robot %d: x = %4.1f, y = %4.1f   \r", robots[0].id, robots[0].x, robots[0].y, robots[1].id, robots[1].x, robots[1].y, robots[2].id, robots[2].x, robots[2].y);
  // }
        
      
  

  // if (mscount >= (300)) //Si on veut imposer un temps d'échantionnage à 2.5 ms par ex
  // {   
  //   Serial.print("erreur temp calcul  :  ");
  //   Serial.println(mscount);
  //   remplirStruct(txMsg, ERREUR_TEMP_CALCUL_LIDAR, 2, mscount,25,0,0,0,0,0,0); //ERREUR_TEMP_CALCUL_LIDAR
  //   writeStructInCAN(txMsg);
  //   mscount = 0;    
  // }
  // else {while (mscount<(300));}
  // // digitalWrite(27, set);//pour mesurer le temps de boucle avec l'oscilloscope
  // // set = !set; //temps de boucle = 1/(freq/2)
  // mscount = 0;
  
  }
}

//Huskylens
void setupHuskylens();
void huskylensLoop();
float Asser_Pos(HUSKYLENSResult result, float pcons_pos);
void printResult(HUSKYLENSResult result);
bool isInside(int value, int min, int max){
    return (value >= min && value <= max);
}
void setup()
{
  Serial.begin(921600);
  Serial.printf("Start\n"); // bind the RPLIDAR driver to the arduino hardware serial
  setupCAN();

  // Initialisation des informations sur les robots détectés
  for (int i = 0; i < MAX_ROBOTS; i++)
  {
    robots[i].id = i + 1;
    robots[i].x = 0;
    robots[i].y = 0;
    robots[i].found = false;
    robots[i].detection_counter = 0;
  }
  initRPLidarA3M1(20000, 8, 255);
  //setupLidar();
  
  //setupHuskylens();

  //init_Timer();

  // Création de la tâche pour l'UART sur le deuxième cœur
  //xTaskCreatePinnedToCore(uartTask, "UART Task", 10000, NULL, 1, NULL, 1);

  mscount = 0;
  mscount2 = 0;
}
bool set = false;
void loop()
{
  CANloop();
  detectionRobots(x_robot, y_robot, theta_robot);
  //Serial.printf("Obstacle %d: x = %4.1f, y = %4.1f    Obstacle %d: x = %4.1f, y = %4.1f     Obstacle %d: x = %4.1f, y = %4.1f   ,  Notre Robot : x = %4.1f, y = %4.1f, theta = %4.1f\n", robots[0].id, robots[0].x, robots[0].y, robots[1].id, robots[1].x, robots[1].y, robots[2].id, robots[2].x, robots[2].y, x_robot, y_robot, theta_robot);
  
 }
















//Lidar

void setupLidar(){
 // Initialisation des informations sur les robots détectés
  for (int i = 0; i < MAX_ROBOTS; i++)
  {
    robots[i].id = i + 1;
    robots[i].x = 0;
    robots[i].y = 0;
    robots[i].found = false;
    robots[i].detection_counter = 0;
  }

  initRPLidarA3M1(20000, 8, 255);
}

void initRPLidarA3M1(int frequence, int resolution, int vitesse)
{
  lidar.begin(Serial2);
  ledcSetup(PWM_CHANNEL, frequence, resolution); // Configuration du canal PWM_CHANNEL avec la fréquence et la résolution choisie
  ledcAttachPin(RPLIDAR_MOTOR, PWM_CHANNEL);     // Assigne le canal PWM au pin 18
  ledcWrite(PWM_CHANNEL, vitesse);               // Créer la tension en sortie choisie
  lidar.startScan();
}

void detectionRobots(float xrobot, float yrobot, float thetarobot)
{
  
  // Attendre la réception d'un nouveau point du Lidar
  if (IS_OK(lidar.waitPoint()))
  {
   
    // Récupération de la distance et de l'angle du point courant
    float distance_lidar = lidar.getCurrentPoint().distance;
    float angle_lidar = lidar.getCurrentPoint().angle - thetarobot - 90;
    float x;
    float y;
     Serial.printf("..");
    /*----------CALCUL DU RESULTAT EN X ET Y----------*/
    if (distance_lidar > 0)
    {
      if (angle_lidar < 0)
      {
        angle_lidar += 360;
      }
      x = xrobot + distance_lidar * sin(angle_lidar * DEG_TO_RAD);
      y = yrobot + distance_lidar * cos(angle_lidar * DEG_TO_RAD);
      /*----------SI LES COORDONNES SONT DANS LE RECTANGLE----------*/
      if (x >= DISTANCE_LIMITE && x <= RECTANGLE_X - DISTANCE_LIMITE && y >= DISTANCE_LIMITE && y <= RECTANGLE_Y - DISTANCE_LIMITE)
      {
        addOrUpdateRobot(x, y);
        //newObject = true;
        Serial.printf("Obstacle %d: x = %4.1f, y = %4.1f    Obstacle %d: x = %4.1f, y = %4.1f     Obstacle %d: x = %4.1f, y = %4.1f   ,  Notre Robot : x = %4.1f, y = %4.1f, theta = %4.1f\n", robots[0].id, robots[0].x, robots[0].y, robots[1].id, robots[1].x, robots[1].y, robots[2].id, robots[2].x, robots[2].y, xrobot, yrobot, thetarobot);
      }
    }
  }
}
// Fonction pour ajouter ou mettre à jour les coordonnées d'un robot
void addOrUpdateRobot(float x, float y)
{
  bool updated = false;          // Variable pour savoir si un robot a été mis à jour
  bool found_empty_slot = false; // Variable pour savoir si un slot vide a été trouvé
  // Parcours de tous les robots dans la structure
  for (int i = 0; i < MAX_ROBOTS; i++)
  {
    // Calcul de la distance entre le robot actuel et le robot précédent
    float distance_precedente = sqrt(pow(x - robots[i].x, 2) + pow(y - robots[i].y, 2));
    // Si le robot actuel a déjà été trouvé et qu'il est proche du précédent
    if (robots[i].found && distance_precedente < 150) // Si le robot a déjà été trouvé et qu'il est à moins d'un rayon de 50 mm de distance
    {
      // On estime que c'est le même robot et on met à jour ses coordonnées
      robots[i].x_history[robots[i].history_index] = x;
      robots[i].y_history[robots[i].history_index] = y;
      robots[i].history_index = (robots[i].history_index + 1) % MOVING_AVG_WINDOW;
      float x_sum = 0;
      float y_sum = 0;
      for (int j = 0; j < MOVING_AVG_WINDOW; j++)
      {
        x_sum += robots[i].x_history[j];
        y_sum += robots[i].y_history[j];
      }
      robots[i].x = x_sum / MOVING_AVG_WINDOW;
      robots[i].y = y_sum / MOVING_AVG_WINDOW;
      robots[i].detection_counter = 0; // Réinitialisation du compteur
      updated = true;
      break;
    }
    else if (!robots[i].found) // Si le robot n'a pas été trouvé
    {
      robots[i].x = x;
      robots[i].y = y;
      robots[i].found = true;
      robots[i].detection_counter = 0; // Réinitialisation du compteur
      robots[i].history_index = 0;
      for (int j = 0; j < MOVING_AVG_WINDOW; j++)
      {
        robots[i].x_history[j] = x;
        robots[i].y_history[j] = y;
      }
      updated = true;
      found_empty_slot = true;
      break;
    }
  }
  // Si aucun robot n'a été mis à jour et qu'il n'y a pas de slot vide
  if (!updated && found_empty_slot == false)
  {
    // On considère qu'un robot a disparu
    for (int i = 0; i < MAX_ROBOTS; i++)
    {
      // Si le robot a été trouvé
      if (robots[i].found)
      {
        robots[i].detection_counter++; // On incrémente le compteur
        // Si on n'a pas détecté le robot depuis 3 cycles de la boucle
        if (robots[i].detection_counter >= 3)
        {
          // On considère que le robot a disparu et on met à jour la structure
          robots[i].found = false;
          break;
        }
      }
    }
  }
  // Serial print pour débugger
  //Serial.printf("Robot %d: x = %4.1f, y = %4.1f    Robot %d: x = %4.1f, y = %4.1f     Robot %d: x = %4.1f, y = %4.1f   \r", robots[0].id, robots[0].x, robots[0].y, robots[1].id, robots[1].x, robots[1].y, robots[2].id, robots[2].x, robots[2].y);
}

// envoi x,y position detectée au bus can 
void envoiCAN(){
  for(int i =0; i < MAX_ROBOTS; i++){
    if(robots[i].found){
      Serial.printf("Found! \n");
      remplirStruct(txMsg, IDCAN_POS_XY_OBJET, 5, robots[i].id, 
                      (short)robots[i].x & 0xFF, ((short)robots[i].x >> 8) & 0xFF, 
                      ((short)robots[i].y & 0xFF), ((short)robots[i].y >> 8) & 0xFF,
                      0,0,0);
      writeStructInCAN(txMsg);//Envoie par bus can du numéro de l'objet et de ses coordonnées x et y. [id][x][x][y]|y]

      if(abs(robots[i].x - x_robot)<120.0 && abs(robots[i].y - y_robot)<120.0 && asservissementEnable){
        asservissementEnable = false;
        Serial.printf("Attention robot %d est trop proche! Envoi requete stop à la base! \n", robots[i].id);
        remplirStruct(txMsg, ASSERVISSEMENT_ENABLE, 0,0,0,0,0,0,0,0,0);//Envoi de l'ordre STOP à la base roulante. Normalement on doit pas faire ça
                                                                      //On doit envoyer les coordonnées de l'objet detecté par bus CAN, comme fait
                                                                      //juste avant, et la carte stratégie en déduira elle meme qu'il faut arreter
                                                                      //la base roulante.
        writeStructInCAN(txMsg);
        // printStruct(txMsg);
      }else if(!asservissementEnable){

      }
    }
  }
}


//CAN
void setupCAN(){
  while (!Serial);
  Serial.println("CAN init");
  // start the CAN bus at 1000 kbps
  if (!CAN.begin(1000E3)) { //ici, nous avons modifié la bibliotheque CAN pour qu'elle soit compatible avec l'ESP32-E
    Serial.println("Starting CAN failed!");
    while (1);
  }
  CAN.onReceive(canReadData); //init CAN callback function
}

void CANloop(){
  
  if (CANAvailable)
    {
      canReadExtRtr();
      CANAvailable = false;
      //printStruct(rxMsg);
      switch (rxMsg.ID)
      {
      case ODOMETRIE_SMALL_POSITION:
      {
        Odo_x = ((rxMsg.dt[1]&0xFF)     << 8)   | (rxMsg.dt[0]&0xFF);
        Odo_y = ((rxMsg.dt[3]&0xFF)     << 8)   | (rxMsg.dt[2]&0xFF);
        Odo_theta = (((rxMsg.dt[5]&0xFF) << 8)   | (rxMsg.dt[4]&0xFF));//en dizieme de degree

        

        x_robot = (float)Odo_x;//Pour le lidar
        y_robot = (float)Odo_y;
        if(Odo_theta>=0){
          theta_robot = (float)Odo_theta / 10.00;//degrees
        }else{
          theta_robot = (float)(Odo_theta * -1) / 10.00 *-1.0;//degrees
        }
        
        currentPos = theta_robot;
        //Serial.printf("CAN currentPos = %4.1f\n", currentPos);

        //Serial.printf("x_robot = %4.1f, y_robot = %4.1f, theta_robot = %4.1f\n", x_robot, y_robot, theta_robot);
      }
      break;

      default:
        break;
      }
    }
}

void canReadData(int packetSize){
  
  rxMsg.ID = CAN.packetId();
  rxMsg.ln = CAN.packetDlc();
  // only print packet data for non-RTR packets
  int i = 0;
  while (CAN.available())
  {
    rxMsg.dt[i]=CAN.read();
    i++;
  }
  CANAvailable = true;
}

void canReadExtRtr(){
  if (CAN.packetExtended()){
      Serial.print("extended ");
      rxMsg.extented = true;
  }else{rxMsg.extented = false;}
  if (CAN.packetRtr()){
      // Remote transmission request, packet contains no data
      Serial.print("RTR ");
      rxMsg.RTR = true;
  }
  else{rxMsg.RTR = false;}
  //Serial.println(" Received CAN ");
}

void printStruct(CANMessage msg)
{
  Serial.printf("id : %0X, len : %d ; data :  ", msg.ID, msg.ln);
  for (int i = 0; i < msg.ln; i++)
  {
    Serial.printf("[%0X]", msg.dt[i]);
  }
  Serial.println("");
}


void writeStructInCAN(const CANMessage &theDATA){
  Serial.print("Sending ");

  if(theDATA.extented){
    CAN.beginExtendedPacket(theDATA.ID, theDATA.ln, theDATA.RTR);
    Serial.print("extended ");
  }
  else{CAN.beginPacket(theDATA.ID, theDATA.ln, theDATA.RTR);}

  Serial.print("packet on CAN... ");

  if(!theDATA.RTR){CAN.write(theDATA.dt, theDATA.ln);}
  
  CAN.endPacket();

  Serial.println("done");
  Serial.println();
}


void remplirStruct(CANMessage &theDATA, int idf, char lenf, char dt0f, char dt1f, char dt2f, char dt3f, char dt4f, char dt5f, char dt6f, char dt7f){
  if(idf>0x7FF){theDATA.extented = true;}
  else{theDATA.extented = false;}
  theDATA.RTR = false;
  theDATA.ID = idf;
  theDATA.ln = lenf;
  theDATA.dt[0] = dt0f;
  theDATA.dt[1] = dt1f;
  theDATA.dt[2] = dt2f;
  theDATA.dt[3] = dt3f;
  theDATA.dt[4] = dt4f;
  theDATA.dt[5] = dt5f;
  theDATA.dt[6] = dt6f;
  theDATA.dt[7] = dt7f;
}

//Huskylens

void setupHuskylens(){
  Serial1.begin(9600,SERIAL_8N1,13,14);//(rx, tx)
  Serial.println(F("Start"));
  if (!huskylens.begin(Serial1))
    {
        Serial.println(F("Begin huskylens failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
    huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING); //Switch the algorithm to object tracking.
}

bool FRONT = false;
void huskylensLoop(){
  if(!huskylens.available()) {
      if (!huskylens.request()) {Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));}
      else if(!huskylens.isLearned()){ Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));}
      else if(!FRONT){Serial.printf("No block or arrow appears on the screen!\n"); FRONT = true;}
    }
    else
    {
        FRONT = false;
        Serial.println(F("###########"));
        if (huskylens.available())
        {
          HUSKYLENSResult hsky = huskylens.read();
          printResult(hsky);
          if(hsky.ID == 1){
            //printResult(hsky);
            Serial.printf("current Pos : %f      ", currentPos);
            short erreur = 160 - hsky.xCenter;
            if(!isInside(erreur, -hsky.width/4, hsky.width/4)){
              currentPos += Asser_Pos(hsky, 160.0);
              char sens = 1;
              short theta = (short)(currentPos*10.0);
              Serial.printf("theta : %d      \n", theta);
              remplirStruct(txMsg, ASSERVISSEMENT_XYT, 7, 
                            (Odo_x & 0xFF), ((Odo_x >> 8) & 0xFF), 
                            (Odo_y & 0xFF), ((Odo_y >> 8) & 0xFF),
                            ((short)currentPos &0xFF), (((short)currentPos >> 8) & 0xFF),
                            sens, 0);
              writeStructInCAN(txMsg);
            }
          }
        }    
    }
}

void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        Serial.println("Object unknown!");
    }
}

float Asser_Pos(HUSKYLENSResult result, float pcons_pos)
{
    float pos, Delta_ErreurPos, commande;
    pos = result.xCenter;                            // Recuperation de la valeur du compteur incrémental
    ErreurPos = pcons_pos - pos;                   // Calcul l'erreur par rapport à la consigne
    Delta_ErreurPos = ErreurPos - last_ErreurPos; // Calcul de la derivee
    Somme_ErreurPos += ErreurPos;                 // Calcul de l'integrale
    if (Kip > 0)
    {
        if (Somme_ErreurPos <= (-250 / Kip))
            Somme_ErreurPos = (-250 / Kip);
        else if (Somme_ErreurPos >= (250 / Kip))
            Somme_ErreurPos = (250 / Kip);
    }
    commande = (Kpp * ErreurPos + Kip * Somme_ErreurPos + Kdp * Delta_ErreurPos); // Mise a jour de la commande moteur
    last_ErreurPos = ErreurPos;    
    Serial.printf("commande : %f\n", commande)  ;                                                 // Stockage de la nouvelle erreur
    return commande;
}

//Timer
void init_Timer(){
  static char idTimer = 0; //le numéro du Timer de 0 à 3
  static int prescaler = 8000; // la valeur du diviseur de temps
  bool flag = true; //vrai pour compter sur le front montant, faux pour compter sur le front descendant
    // Configure le Prescaler a 80 le quartz de l ESP32 est cadence a 80Mhz => à vérifier pour l'esp32-32E, peut etre 40Mhz?
   // 80000000 / 80 = 1000000 tics / seconde
   timer = timerBegin(idTimer, prescaler, flag);
   timerAttachInterrupt(timer, &onTime, true);//fait qu'on execute la fonction onTime à chaque interruptions
    
   // Regle le declenchement d une alarme chaque seconde
   timerAlarmWrite(timer, 1, true);      //freq de 250 000 Hz    
   timerAlarmEnable(timer); //active l'alarme
   Serial.println("Fin init Timer");
}
void onTime() {//fonction s'exécutent à chaque interruptions 
   mscount++;
   mscount2 ++;
   
   
}