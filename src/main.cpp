#include <Arduino.h>
#include <CAN/src/CAN.h>
#include <RPLidar.h>
#include <math.h>

#include "ident_crac.h"

// #define RPLIDAR_MOTOR 15      // Pin du moteur du Lidar
// Vieille carte broche différente : 26
#define RPLIDAR_MOTOR 23      // Pin du moteur du Lidar

#define PWM_CHANNEL 0         // Canal PWM pour le contrôle du moteur
#define RECTANGLE_X 2000.0f   // Largeur du terrain en mm
#define RECTANGLE_Y 3000.0f   // Hauteur du terrain en mm
#define DISTANCE_LIMITE 120   // Marge de sécurité pour les bords du terrain
#define ANGLE_RESOLUTION 360  // Nombre de points que l'on prend sur un tour du lidar

bool startBit = false;
float distanceData[ANGLE_RESOLUTION] = {0};  // Tableau pour stocker les distances du lidar

#define MAX_ROBOTS 3         // Nombre maximal de robots à détecter
#define MOVING_AVG_WINDOW 5  // Taille de la fenêtre pour la moyenne mobile

// Structure pour stocker les informations sur les robots détectés
struct RobotInfo {
    int id;   // Identifiant du robot (de 1 à 3)
    float x;  // Position en X du robot
    float y;  // Position en Y du robot
    float theta;
    float distance_lidar;
    bool found;                          // Indique si le robot a été trouvé lors de la dernière itération
    int detection_counter;               // Compteur pour estimer la fiabilité de la détection
    float x_history[MOVING_AVG_WINDOW];  // Historique des positions en X pour la moyenne mobile
    float y_history[MOVING_AVG_WINDOW];  // Historique des positions en Y pour la moyenne mobile
    int history_index;                   // Index courant dans les historiques de positions
};
RobotInfo robots[MAX_ROBOTS];  // Tableau pour stocker les informations sur les robots détectés
void initRPLidarA3M1(int frequence, int resolution, int vitesse);
void detectionRobots(float x_robot, float y_robot, float theta_robot);
void analyse(void);
void addOrUpdateRobot(float x, float y, float theta, float distance);
RPLidar lidar;

// CAN-----------------------------------------------
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

// Valeur du robot envoyé par le CAN
float x_robot;
float y_robot;
float theta_robot;

short Odo_x = 0, Odo_y = 0, Odo_theta = 0;
bool asservissementEnable = true;

// CAN
void setupCAN();
void CANloop();
void canReadData(int packetSize);
void canReadExtRtr();
void writeStructInCAN(const CANMessage &theDATA);
void printStruct(CANMessage msg);
void remplirStruct(CANMessage &theDATA, int idf, char lenf,
                   char dt0f, char dt1f, char dt2f, char dt3f,
                   char dt4f, char dt5f, char dt6f, char dt7f);
void envoiCAN();

//----------------------------------------------------------------------Timer

int totalInterrupts = 0;  // compte le nombre de declenchement de l alarme
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
uint16_t mscount /*temps d'echantionnage*/ = 0, mscount2 /*bus can*/ = 0;
void init_Timer();
void onTime();  // prototype de la fonction s'exécutent à chaque interruptions

void setup() {
    Serial.begin(921600);  // Initialisation de la communication série pour le debug
    setupCAN();
    Serial.println("\nStart...");

    // Initialisation des informations sur les robots détectés
    for (int i = 0; i < MAX_ROBOTS; i++) {
        robots[i].id = i + 1;
        robots[i].x = 0;
        robots[i].y = 0;
        robots[i].theta = 0;
        robots[i].found = false;
        robots[i].detection_counter = 0;
    }
    delay(2000);
    initRPLidarA3M1(20000, 8, 128);
    init_Timer();
}

float angle_memo = 0;

void loop() {
    CANloop();

    // if (IS_OK(lidar.waitPoint()))
    // {
    //   __uint8_t quality = lidar.getCurrentPoint().quality;
    //   float distance_mesure = lidar.getCurrentPoint().distance;
    //   float angle_mesure = lidar.getCurrentPoint().angle;
    //   Serial.printf(" %f   %f \n",angle_mesure, distance_mesure);
    //   if (angle_mesure < angle_memo)
    //   {
    //     analyse();
    //   }
    //   angle_memo = angle_mesure;
    //   int index = angle_mesure;
    //   distanceData[index] = distance_mesure;
    // }
    if (IS_OK(lidar.waitPoint())) {
        startBit = lidar.getCurrentPoint().startBit;

        if (startBit) {
            for (int i = 0; i < ANGLE_RESOLUTION; i++) {
                distanceData[i] = -1;
            }
            angle_memo = 0;

            while (1) {
                if (IS_OK(lidar.waitPoint())) {
                    float angle_mesure = lidar.getCurrentPoint().angle;
                    if (angle_mesure < 10) {
                        break;
                    }
                }
            }

            while (1) {
                __uint8_t quality = lidar.getCurrentPoint().quality;
                float distance_mesure = lidar.getCurrentPoint().distance;
                float angle_mesure = lidar.getCurrentPoint().angle;
                int index = angle_mesure;
                // Serial.printf(" %f   %f\n", angle_mesure, distance_mesure);
                distanceData[index] = distance_mesure;
                // Serial.printf("%d %f\n", index, distance_mesure);
                if (IS_OK(lidar.waitPoint())) {
                    float angle_mesure = lidar.getCurrentPoint().angle;
                    if (angle_memo - angle_mesure > 100) {
                        break;
                    }
                    angle_memo = angle_mesure;
                }
            }
            // Traitement tableau

            for (int i = 0; i < ANGLE_RESOLUTION; i++) {
                if ((i > 45 && i < 135) || (i > 225 && i < 315) || (distanceData[i] <= 100) || (distanceData[i] > 1000)) {
                    distanceData[i] = 10000;
                } else {
                    float x = x_robot + distanceData[i] * cos((theta_robot-i) * DEG_TO_RAD);
                    float y = y_robot + distanceData[i] * sin((theta_robot-i) * DEG_TO_RAD);

                    /*----------SI LES COORDONNES SONT DANS LE RECTANGLE----------*/
                    if (x >= DISTANCE_LIMITE && x <= RECTANGLE_X - DISTANCE_LIMITE && y >= DISTANCE_LIMITE && y <= RECTANGLE_Y - DISTANCE_LIMITE) {
                        // data ok
                    } else {
                      distanceData[i] = 10000;
                    }
                }
//                Serial.printf("%d %f\n", i, distanceData[i]);
            }
            int minAvantDist = 10000;
            int angleAvant = 40;
            int minArriereDist = 10000;
            int angleArriere = 140;
            for (int i=0; i<=45; i++) {
              if (distanceData[i]<minAvantDist) {
                minAvantDist = distanceData[i];
                angleAvant = 360-i;
              }
            }
            for (int i=315; i<360; i++) {
              if (distanceData[i]<minAvantDist) {
                minAvantDist = distanceData[i];
                angleAvant = 360-i;
              }
            }
            for (int i=135; i<=225; i++) {
              if (distanceData[i]<minArriereDist) {
                minArriereDist = distanceData[i];
                angleArriere = 360-i;
              }
            }
            if (minAvantDist < 10000) {
              // envoi CAN
              remplirStruct(txMsg, IDCAN_POS_XY_OBJET, 5, 1,
                          (unsigned short)minAvantDist & 0xFF, ((unsigned short)minAvantDist >> 8) & 0xFF,
                          (unsigned short)(angleAvant * 10) & 0xFF, ((unsigned short)(angleAvant * 10) >> 8) & 0xFF,  // en diziéme de degree
                          0, 0, 0);
              writeStructInCAN(txMsg);
            }
            if (minArriereDist < 10000) {
              // envoi CAN
              remplirStruct(txMsg, IDCAN_POS_XY_OBJET, 5, 1,
                          (unsigned short)minArriereDist & 0xFF, ((unsigned short)minArriereDist >> 8) & 0xFF,
                          (unsigned short)(angleArriere * 10) & 0xFF, ((unsigned short)(angleArriere * 10) >> 8) & 0xFF,  // en diziéme de degree
                          0, 0, 0);
              writeStructInCAN(txMsg);              
            }
        }
    }
    // if (mscount2 > 350)
    // {
    //   mscount2 = 0;
    //   envoiCAN();
    //   for (int i = 0; i < MAX_ROBOTS; i++)
    //   {
    //     robots[i].id = i + 1;
    //     robots[i].x = 0;
    //     robots[i].y = 0;
    //     robots[i].theta = 0;
    //     robots[i].found = false;
    //     robots[i].detection_counter = 0;
    //   }
    // }

    // detectionRobots(x_robot, y_robot, theta_robot);
}

void initRPLidarA3M1(int frequence, int resolution, int vitesse) {
    lidar.begin(Serial2);
    ledcSetup(PWM_CHANNEL, frequence, resolution);  // Configuration du canal PWM_CHANNEL avec la fréquence et la résolution choisie
    ledcAttachPin(RPLIDAR_MOTOR, PWM_CHANNEL);      // Assigne le canal PWM au pin 18
    ledcWrite(PWM_CHANNEL, vitesse);                // Créer la tension en sortie choisie
    lidar.startScan();
}

// void detectionRobots(float x_robot, float y_robot, float theta_robot)
// {
//   startBit = lidar.getCurrentPoint().startBit;

//   if (startBit)
//   {
//     for (int i = 0;; i++)
//     {
//       if (IS_OK(lidar.waitPoint()))
//       {
//         __uint8_t quality = lidar.getCurrentPoint().quality;
//         float distance_mesure = lidar.getCurrentPoint().distance;
//         float angle_mesure = lidar.getCurrentPoint().angle;
//         int index = round(angle_mesure);

//         if (index >= i)
//         {
//           distanceData[index] = distance_mesure;
//         }
//       }
//     }

//     float x;
//     float y;
//     /*----------CALCUL DU RESULTAT EN X ET Y----------*/
//     if (distance_lidar > DISTANCE_LIMITE && quality >= 6)
//     {
//       if (angle_lidar < 0)
//       {
//         angle_lidar += 360;
//       }
//       else if (angle_lidar >= 360)
//       {
//         angle_lidar -= 360;
//       }

//       x = x_robot + distance_lidar * cos(angle_lidar * DEG_TO_RAD);
//       y = y_robot + distance_lidar * sin(angle_lidar * DEG_TO_RAD);

//       /*----------SI LES COORDONNES SONT DANS LE RECTANGLE----------*/

//       if (x >= DISTANCE_LIMITE && x <= RECTANGLE_X - DISTANCE_LIMITE && y >= DISTANCE_LIMITE && y <= RECTANGLE_Y - DISTANCE_LIMITE)
//       {
//         addOrUpdateRobot(x, y, 360 - angle_mesure, distance_lidar);
//       }
//     }
//   }
// }

void analyse(void) {
    for (int i = 0; i < ANGLE_RESOLUTION; i++) {
        Serial.printf("[%d] dist [%f]\n", i, distanceData[i]);
    }
}

// Fonction pour ajouter ou mettre à jour les coordonnées d'un robot
// void addOrUpdateRobot(float x, float y, float theta, float distance)
// {
//   bool updated = false;          // Variable pour savoir si un robot a été mis à jour
//   bool found_empty_slot = false; // Variable pour savoir si un slot vide a été trouvé
//   // Parcours de tous les robots dans la structure
//   for (int i = 0; i < MAX_ROBOTS; i++)
//   {
//     // Calcul de la distance entre le robot actuel et le robot précédent
//     float distance_precedente = sqrt(pow(x - robots[i].x, 2) + pow(y - robots[i].y, 2));
//     // Si le robot actuel a déjà été trouvé et qu'il est proche du précédent
//     if (robots[i].found && distance_precedente < 50) // Si le robot a déjà été trouvé et qu'il est à moins d'un rayon de 50 mm de distance
//     {
//       // On estime que c'est le même robot et on met à jour ses coordonnées
//       robots[i].x_history[robots[i].history_index] = x;
//       robots[i].y_history[robots[i].history_index] = y;
//       robots[i].history_index = (robots[i].history_index + 1) % MOVING_AVG_WINDOW;
//       float x_sum = 0;
//       float y_sum = 0;
//       for (int j = 0; j < MOVING_AVG_WINDOW; j++)
//       {
//         x_sum += robots[i].x_history[j];
//         y_sum += robots[i].y_history[j];
//       }
//       robots[i].x = x_sum / MOVING_AVG_WINDOW;
//       robots[i].y = y_sum / MOVING_AVG_WINDOW;
//       robots[i].theta = theta;
//       robots[i].distance_lidar = distance_lidar;
//       robots[i].detection_counter = 0; // Réinitialisation du compteur
//       updated = true;
//       break;
//     }
//     else if (!robots[i].found && (x >= DISTANCE_LIMITE && x <= RECTANGLE_X - DISTANCE_LIMITE && y >= DISTANCE_LIMITE && y <= RECTANGLE_Y - DISTANCE_LIMITE)) // Si le robot n'a pas été trouvé
//     {
//       robots[i].x = x;
//       robots[i].y = y;
//       robots[i].theta = theta;
//       robots[i].distance_lidar = distance_lidar;
//       robots[i].found = true;
//       robots[i].detection_counter = 0; // Réinitialisation du compteur
//       robots[i].history_index = 0;
//       for (int j = 0; j < MOVING_AVG_WINDOW; j++)
//       {
//         robots[i].x_history[j] = x;
//         robots[i].y_history[j] = y;
//       }
//       updated = true;
//       found_empty_slot = true;
//       break;
//     }
//   }
//   // Si aucun robot n'a été mis à jour et qu'il n'y a pas de slot vide
//   if (!updated && found_empty_slot == false)
//   {
//     // On considère qu'un robot a disparu
//     for (int i = 0; i < MAX_ROBOTS; i++)
//     {
//       // Si le robot a été trouvé
//       if (robots[i].found)
//       {
//         robots[i].detection_counter++; // On incrémente le compteur
//         // Si on n'a pas détecté le robot depuis 3 cycles de la boucle
//         if (robots[i].detection_counter >= 3)
//         {
//           // On considère que le robot a disparu et on met à jour la structure
//           robots[i].found = false;
//           break;
//         }
//       }
//     }
//   }
//   // Serial print pour débugger
//   // Serial.printf("Obstacle %d: x = %4.1f, y = %4.1f    Obstacle %d: x = %4.1f, y = %4.1f     Obstacle %d: x = %4.1f, y = %4.1f   ,  Notre Robot : x = %4.1f, y = %4.1f, theta = %4.1f\n", robots[0].id, robots[0].x, robots[0].y, robots[1].id, robots[1].x, robots[1].y, robots[2].id, robots[2].x, robots[2].y, x_robot, y_robot, theta_robot);
// }

// envoi x,y position detectée au bus can
void envoiCAN() {
    for (int i = 0; i < MAX_ROBOTS; i++) {
        if (robots[i].found) {
            remplirStruct(txMsg, IDCAN_POS_XY_OBJET, 5, robots[i].id,
                          (unsigned short)robots[i].distance_lidar & 0xFF, ((unsigned short)robots[i].distance_lidar >> 8) & 0xFF,
                          (unsigned short)(robots[i].theta * 10.0) & 0xFF, ((unsigned short)(robots[i].theta * 10.0) >> 8) & 0xFF,  // en diziéme de degree
                          0, 0, 0);
            writeStructInCAN(txMsg);
        }
    }
}

// CAN
void setupCAN() {
    while (!Serial)
        ;
    Serial.println("CAN init");
    // start the CAN bus at 1000 kbps
    if (!CAN.begin(1000E3)) {  // ici, nous avons modifié la bibliotheque CAN pour qu'elle soit compatible avec l'ESP32-E
        Serial.println("Starting CAN failed!");
        while (1)
            ;
    }
    CAN.onReceive(canReadData);  // init CAN callback function
}

void CANloop() {
    if (CANAvailable) {
        //    canReadExtRtr();
        CANAvailable = false;
        // printStruct(rxMsg);
        //    switch (rxMsg.ID)
        //    {
        //    case ODOMETRIE_SMALL_POSITION:
        //    {
        Odo_x = ((rxMsg.dt[1] & 0xFF) << 8) | (rxMsg.dt[0] & 0xFF);
        Odo_y = ((rxMsg.dt[3] & 0xFF) << 8) | (rxMsg.dt[2] & 0xFF);
        Odo_theta = (((rxMsg.dt[5] & 0xFF) << 8) | (rxMsg.dt[4] & 0xFF));  // en dizieme de degree

        x_robot = (float)Odo_x;  // Pour le lidar
        y_robot = (float)Odo_y;
        if (Odo_theta >= 0) {
            theta_robot = (float)Odo_theta / 10.00;  // degrees
        } else {
            theta_robot = (float)(Odo_theta * -1) / 10.00 * -1.0;  // degrees
        }

        // currentPos = theta_robot;
        // Serial.printf("CAN currentPos = %4.1f\n", currentPos);

        // Serial.printf("x_robot = %4.1f, y_robot = %4.1f, theta_robot = %4.1f\n", x_robot, y_robot, theta_robot);
        //   }
        //   break;
        //   default:
        //     break;
        //   }
    }
}

void canReadData(int packetSize) {
    rxMsg.ID = CAN.packetId();
    if (rxMsg.ID != ODOMETRIE_SMALL_POSITION)
        return;
    rxMsg.ln = CAN.packetDlc();
    // only print packet data for non-RTR packets
    int i = 0;
    while (CAN.available()) {
        rxMsg.dt[i] = CAN.read();
        i++;
    }
    CANAvailable = true;
}

void canReadExtRtr() {
    if (CAN.packetExtended()) {
        Serial.print("extended ");
        rxMsg.extented = true;
    } else {
        rxMsg.extented = false;
    }
    if (CAN.packetRtr()) {
        // Remote transmission request, packet contains no data
        Serial.print("RTR ");
        rxMsg.RTR = true;
    } else {
        rxMsg.RTR = false;
    }
    // Serial.println(" Received CAN ");
}

void printStruct(CANMessage msg) {
    Serial.printf("id : %0X, len : %d ; data :  ", msg.ID, msg.ln);
    for (int i = 0; i < msg.ln; i++) {
        Serial.printf("[%0X]", msg.dt[i]);
    }
    Serial.println("");
}

void writeStructInCAN(const CANMessage &theDATA) {
    // Serial.print("Sending ");

    if (theDATA.extented) {
        CAN.beginExtendedPacket(theDATA.ID, theDATA.ln, theDATA.RTR);
        // Serial.print("extended ");
    } else {
        CAN.beginPacket(theDATA.ID, theDATA.ln, theDATA.RTR);
    }

    // Serial.print("packet on CAN... ");

    if (!theDATA.RTR) {
        CAN.write(theDATA.dt, theDATA.ln);
    }

    CAN.endPacket();

    // Serial.println("done");
    // Serial.println();
}

void remplirStruct(CANMessage &theDATA, int idf, char lenf, char dt0f, char dt1f, char dt2f, char dt3f, char dt4f, char dt5f, char dt6f, char dt7f) {
    if (idf > 0x7FF) {
        theDATA.extented = true;
    } else {
        theDATA.extented = false;
    }
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

// Timer
void init_Timer() {
    static char idTimer = 0;      // le numéro du Timer de 0 à 3
    static int prescaler = 8000;  // la valeur du diviseur de temps
    bool flag = true;             // vrai pour compter sur le front montant, faux pour compter sur le front descendant
                                  //  Configure le Prescaler a 80 le quartz de l ESP32 est cadence a 80Mhz => à vérifier pour l'esp32-32E, peut etre 40Mhz?
    // 80000000 / 80 = 1000000 tics / seconde
    timer = timerBegin(idTimer, prescaler, flag);
    timerAttachInterrupt(timer, &onTime, true);  // fait qu'on execute la fonction onTime à chaque interruptions

    // Regle le declenchement d une alarme chaque seconde
    timerAlarmWrite(timer, 10, true);  // freq de 250 000 Hz
    timerAlarmEnable(timer);           // active l'alarme
    Serial.println("Fin init Timer");
    mscount = 0;
    mscount2 = 0;
}
void onTime() {  // fonction s'exécutent à chaque interruptions
    mscount++;
    mscount2++;
}

// void initServo(int pin){
//   ESP32PWM::allocateTimer(0);
// 	ESP32PWM::allocateTimer(1);
// 	ESP32PWM::allocateTimer(2);
// 	ESP32PWM::allocateTimer(3);
//     servo_cerise.attach(pin);
// }
