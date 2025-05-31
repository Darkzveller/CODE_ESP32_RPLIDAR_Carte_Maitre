#include <Arduino.h>
#include <Variable.h>
#include <CAN_ESP32E.h>
#include <ID_CAN.h>
#include <USE_FUNCTION.h>
#include <RPLidar.h>
#include <math.h>
#include "STRATEGIE.h"
#include <Wire.h>
#include "rgb_lcd.h"
rgb_lcd lcd;

volatile bool finMvt = false; // Variable pour indiquer si le mouvement est terminé

static unsigned long startDetectionAvant = 0;
static bool objetPresentAvant = false;

static unsigned long startDetectionArriere = 0;
static bool objetPresentArriere = false;

#define RPLIDAR_MOTOR 23 // Pin du moteur du Lidar

#define PWM_CHANNEL 0        // Canal PWM pour le contrôle du moteur
#define RECTANGLE_X 3000.0f  // Largeur du terrain en mm
#define RECTANGLE_Y 2000.0f  // Hauteur du terrain en mm
#define DISTANCE_LIMITE 120  // Marge de sécurité pour les bords du terrain
#define ANGLE_RESOLUTION 360 // Nombre de points que l'on prend sur un tour du lidar

bool startBit = false;
float distanceData[ANGLE_RESOLUTION] = {0}; // Tableau pour stocker les distances du lidar

#define MAX_ROBOTS 1        // Nombre maximal de robots à détecter
#define MOVING_AVG_WINDOW 5 // Taille de la fenêtre pour la moyenne mobile

TaskHandle_t hStrategie = NULL; // Handle pour la tâche de gestion de la stratégie

// Structure pour stocker les informations sur les robots détectés
struct RobotInfo
{
    int id;  // Identifiant du robot (de 1 à 3)
    float x; // Position en X du robot
    float y; // Position en Y du robot
    float theta;
    float distance_lidar;
    bool found;                         // Indique si le robot a été trouvé lors de la dernière itération
    int detection_counter;              // Compteur pour estimer la fiabilité de la détection
    float x_history[MOVING_AVG_WINDOW]; // Historique des positions en X pour la moyenne mobile
    float y_history[MOVING_AVG_WINDOW]; // Historique des positions en Y pour la moyenne mobile
    int history_index;                  // Index courant dans les historiques de positions
};
RobotInfo robots[MAX_ROBOTS]; // Tableau pour stocker les informations sur les robots détectés
void initRPLidarA3M1(int frequence, int resolution, int vitesse);
void detectionRobots(float x_robot, float odo_y_robot, float theta_robot);
void analyse(void);
void addOrUpdateRobot(float x, float y, float theta, float distance);
RPLidar lidar;

void bus_can(void *parameters)
{

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        readCANMessage();

        switch (rxMsg.id)
        {
        case ODO_SEND:

            odo_x_robot = fusion_octet(rxMsg.data[0], rxMsg.data[1]);
            odo_y_robot = fusion_octet(rxMsg.data[2], rxMsg.data[3]);
            theta_robot = fusion_octet(rxMsg.data[4], rxMsg.data[5]);

            // Serial.printf("%hd %hd %hd\n", odo_x_robot, odo_y_robot, theta_robot);

            rxMsg.id = 0;

            break;
        case 0x35: // ID pour la fin de mouvement
            finMvt = true;
            break;

        case 0:

            break;
        default:
            break;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Tcan));
    }
}

void timerFinMatch(void *parameters)
{
    delay(99000);
    vTaskSuspend(hStrategie);
    sendCANMessage(INTERRUPTEUR_BATT1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0);
    sendCANMessage(INTERRUPTEUR_BATT2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0);
    sendCANMessage(INTERRUPTEUR_BATT3, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0);
    while (1)
    {
        delay(1000);
    }
}

void gestion_strategie(void *parameters)
{

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    sendCANMessage(ESP32_RESTART, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    while (1)
    {
        start_match = digitalRead(PIN_READ_FDC);
        static bool choix_couleur_equipe = digitalRead(PIN_CHOIX_COULEUR);
        // Serial.printf("Val couleur ");
        // Serial.print(choix_couleur_equipe)
        //     Serial.printf("start_match %d ", start_match);
        // Serial.println();
        //        Serial.printf("etat %d\n", etat_match);

#ifdef HOMOLOGATION
        switch (etat_match)
        {
        case 0:
            if (choix_couleur_equipe == 0)
            {
                lcd.clear();
                lcd.setRGB(255, 255, 0);
            }
            else
            {
                lcd.clear();
                lcd.setRGB(0, 0, 255);
            }

            if (start_match)
            {
                // Serial.printf("Start match ");
                // Serial.println(start_match);

                etat_match = 1;

                xTaskCreate(
                    timerFinMatch,   // nom de la fonction
                    "timerFinMatch", // nom de la tache que nous venons de vréer
                    10000,           // taille de la pile en octet
                    NULL,            // parametre
                    11,              // tres haut niveau de priorite
                    NULL             // descripteur
                );
            }
            else
            {

                sendCANMessage(INTERRUPTEUR_BATT1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
                sendCANMessage(INTERRUPTEUR_BATT2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
                sendCANMessage(INTERRUPTEUR_BATT3, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
            }
            break;
        case 1:
            sendCANMessage(START_ROBOT_MATCH, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0);

            if (choix_couleur_equipe == 0)
            {
                Serial.printf("Strat jaune + recalage ");
                strategie_jaune_homologation();
            }
            else
            {
                Serial.printf("Strat bleu + recalage ");
                strategie_bleu_homologation();
            }
            Serial.printf("Start match ");
            Serial.printf("Message envoyé et début de match \n");
            etat_match = 2;
            break;

        case 2:
            break;

        default:
            break;
        }

#endif

#ifdef UN_SPOT
        switch (etat_match)
        {
        case 0:

            if (start_match)
            {
                Serial.printf("Start match ");
                Serial.println(start_match);

                etat_match = 1;
            }
            break;
        case 1:
            Serial.printf("Esp restart ");
            sendCANMessage(ESP32_RESTART, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            // Metttre les 5 secondes pour laisser le temps au robot de s'immobiliser
            delay(5000);
            if (choix_couleur_equipe == 0)
            {
                Serial.printf("recalage jaune");
                send_recalage(1225, 250, 90);
            }
            else
            {
                Serial.printf("recalage bleu");
                send_recalage(1775, 250, 90);
            }
            Serial.printf("Start match ");

            sendCANMessage(START_ROBOT_MATCH, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0);
            delay(TIME_TO_SEND_DATA_CAN);
            Serial.printf("ON1 ");
            sendCANMessage(INTERRUPTEUR_BATT1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
            delay(TIME_TO_SEND_DATA_CAN);
            Serial.printf("ON2 ");
            sendCANMessage(INTERRUPTEUR_BATT2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
            delay(TIME_TO_SEND_DATA_CAN);
            Serial.printf("ON3 ");
            sendCANMessage(INTERRUPTEUR_BATT3, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
            delay(TIME_TO_SEND_DATA_CAN);

            sendCANMessage(CONSTRUIRE_AVANT_PREPARER, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
            delay(TIME_TO_SEND_DATA_CAN);

            etat_match = 2;
            break;
        case 2:
            send_ligne_droite(150);
            etat_match = 3;
            break;
        case 3:
            if (rxMsg.id == ACKNOWLEDGE_BASE_ROULANTE)
            {
                rxMsg.id = 0;

                delay(1000);

                send_rotation(-45);

                etat_match = 4;
            }
            break;
        case 4:
            if (rxMsg.id == ACKNOWLEDGE_BASE_ROULANTE)
            {
                rxMsg.id = 0;
                delay(1000);
                send_ligne_droite(175);
                etat_match = 5;
            }
            break;
        case 5:
            if (rxMsg.id == ACKNOWLEDGE_BASE_ROULANTE)
            {
                rxMsg.id = 0;

                delay(1000);
                send_rotation(40);
                etat_match = 6;
            }

        case 6:
            if (rxMsg.id == ACKNOWLEDGE_BASE_ROULANTE)
            {
                rxMsg.id = 0;

                delay(1000);
                send_ligne_droite(600);
                etat_match = 7;
            }

            break;

        case 7:
            if (rxMsg.id == ACKNOWLEDGE_BASE_ROULANTE)
            {
                rxMsg.id = 0;

                send_x_y_theta(600, 1500, 0);
                delay(200);
                sendCANMessage(CONSTRUIRE_AVANT_2ETAGE, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);

                etat_match = 8;
            }

            break;
        case 8:
            break;

        default:
            break;
        }

#endif

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}

bool objetDetect = false;

short Odo_x = 0, Odo_y = 0, Odo_theta = 0;
bool asservissementEnable = true;

//----------------------------------------------------------------------Timer

int totalInterrupts = 0; // compte le nombre de declenchement de l alarme
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
uint16_t mscount /*temps d'echantionnage*/ = 0, mscount2 /*bus can*/ = 0;
void init_Timer();
void onTime(); // prototype de la fonction s'exécutent à chaque interruptions
static int detect_devant = 0;
static int detect_derriere = 0;

void setup()
{
    Serial.begin(115200); // Initialisation de la communication série pour le debug
    Serial.println("\nStart...");

    // Initialisation des informations sur les robots détectés
    for (int i = 0; i < MAX_ROBOTS; i++)
    {
        robots[i].id = i + 1;
        robots[i].x = 0;
        robots[i].y = 0;
        robots[i].theta = 0;
        robots[i].found = false;
        robots[i].detection_counter = 0;
    }
    delay(2000);
    // Initialisation du Can
    setupCAN(1000E3);
    initRPLidarA3M1(20000, 8, 128);
    init_Timer();
    // Initialisation LCD RGB
    lcd.begin(16, 2);
    // Print a message to the LCD.
    lcd.setRGB(255, 255, 255);
    setup_pin_strategie();
    xTaskCreate(
        bus_can,   // nom de la fonction
        "bus_can", // nom de la tache que nous venons de vréer
        10000,     // taille de la pile en octet
        NULL,      // parametre
        9,         // tres haut niveau de priorite
        NULL       // descripteur
    );
    xTaskCreate(
        gestion_strategie,   // nom de la fonction
        "gestion_strategie", // nom de la tache que nous venons de vréer
        10000,               // taille de la pile en octet
        NULL,                // parametre
        10,                  // tres haut niveau de priorite
        &hStrategie          // descripteur
    );
}

float angle_memo = 0;

void loop()
{
    static int i = 0;
    // Serial.printf("i = %d\n", i++);
    if (IS_OK(lidar.waitPoint()))
    {
        startBit = lidar.getCurrentPoint().startBit;

        if (startBit)
        {
            for (int i = 0; i < ANGLE_RESOLUTION; i++)
            {
                distanceData[i] = -1;
            }
            angle_memo = 0;

            while (1)
            {
                if (IS_OK(lidar.waitPoint()))
                {
                    float angle_mesure = lidar.getCurrentPoint().angle;
                    if (angle_mesure < 10)
                    {
                        break;
                    }
                }
            }

            while (1)
            {
                __uint8_t quality = lidar.getCurrentPoint().quality;
                float distance_mesure = lidar.getCurrentPoint().distance;
                float angle_mesure = lidar.getCurrentPoint().angle;
                int index = angle_mesure;
                // Serial.printf("Angle mesure et distance mesure ");
                // Serial.printf(" %f   %f\n", angle_mesure, distance_mesure);
                distanceData[index] = distance_mesure;
                // Serial.printf("%d %f\n", index, distance_mesure);
                if (IS_OK(lidar.waitPoint()))
                {
                    float angle_mesure = lidar.getCurrentPoint().angle;
                    if (angle_memo - angle_mesure > 100)
                    {
                        break;
                    }
                    angle_memo = angle_mesure;
                }
            }
            // Traitement tableau

            for (int i = 0; i < ANGLE_RESOLUTION; i++)
            {
                if ((i > 45 && i < 135) || (i > 225 && i < 315) || (distanceData[i] <= 100) || (distanceData[i] > 750))
                {
                    distanceData[i] = 10000;
                }
                else
                {
                    float x = odo_x_robot + distanceData[i] * cos((theta_robot - i) * DEG_TO_RAD);
                    float y = odo_y_robot + distanceData[i] * sin((theta_robot - i) * DEG_TO_RAD);

                    /*----------SI LES COORDONNES SONT DANS LE RECTANGLE----------*/
                    if (x >= DISTANCE_LIMITE && x <= RECTANGLE_X - DISTANCE_LIMITE && y >= DISTANCE_LIMITE && y <= RECTANGLE_Y - DISTANCE_LIMITE)
                    {
                        // data ok
                    }
                    else
                    {
                        distanceData[i] = 10000;
                    }
                }
                //                Serial.printf("%d %f\n", i, distanceData[i]);
            }
            int minAvantDist = 10000;
            int angleAvant = 40;
            int minArriereDist = 10000;
            int angleArriere = 140;
            for (int i = 0; i <= 45; i++)
            {
                if (distanceData[i] < minAvantDist)
                {
                    minAvantDist = distanceData[i];
                    angleAvant = 360 - i;
                }
            }
            for (int i = 315; i < 360; i++)
            {
                if (distanceData[i] < minAvantDist)
                {
                    minAvantDist = distanceData[i];
                    angleAvant = 360 - i;
                }
            }
            for (int i = 135; i <= 225; i++)
            {
                if (distanceData[i] < minArriereDist)
                {
                    minArriereDist = distanceData[i];
                    angleArriere = 360 - i;
                }
            }
            /*
            if (minAvantDist < 10000)
            {
                Serial.printf("detect_devant %d \n", detect_devant++);

                sendCANMessage(LIDAR, 0, 0, 8, AVANT_LIDAR_DETECTER, 0, 0, 0, 0, 0, 0, 0);
                // envoi CAN
                //   remplirStruct(txMsg, IDCAN_POS_XY_OBJET, 5, 1,
                //               (unsigned short)minAvantDist & 0xFF, ((unsigned short)minAvantDist >> 8) & 0xFF,
                //               (unsigned short)(angleAvant * 10) & 0xFF, ((unsigned short)(angleAvant * 10) >> 8) & 0xFF,  // en diziéme de degree
                //               0, 0, 0);
                //   writeStructInCAN(txMsg);
            }
            if (minArriereDist < 10000)
            {
                Serial.printf("detect_derrire %d \n", detect_derriere++);
                sendCANMessage(LIDAR, 0, 0, 8, ARRIERE_LIDAR_DETECTER, 0, 0, 0, 0, 0, 0, 0);

                // envoi CAN
                //   remplirStruct(txMsg, IDCAN_POS_XY_OBJET, 5, 1,
                //               (unsigned short)minArriereDist & 0xFF, ((unsigned short)minArriereDist >> 8) & 0xFF,
                //               (unsigned short)(angleArriere * 10) & 0xFF, ((unsigned short)(angleArriere * 10) >> 8) & 0xFF,  // en diziéme de degree
                //               0, 0, 0);
                //   writeStructInCAN(txMsg);
            }*/
            if (minAvantDist < 10000)
            {
                // if (!objetPresentAvant)
                // {
                objetPresentAvant = true;
                //     startDetectionAvant = millis();
                // }
                // else if (millis() - startDetectionAvant >= 50)
                // {
                Serial.printf("Objet devant\n");
                sendCANMessage(LIDAR, 0, 0, 2, AVANT_LIDAR_DETECTER, 0, 0, 0, 0, 0, 0, 0);
                // }
            }
            else
            {
                if (objetPresentAvant)
                {
                    Serial.printf("Fin objet devant\n");
                    // delay(1000);
                    sendCANMessage(LIDAR, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0);
                }
                objetPresentAvant = false;
            }
            if (minArriereDist < 10000)
            {
                // if (!objetPresentArriere)
                // {
                objetPresentArriere = true;
                //     startDetectionArriere = millis();
                // }
                // else if (millis() - startDetectionArriere >= 50)
                // {
                Serial.printf("Objet derrière\n");
                //                    sendCANMessage(LIDAR, 0, 0, 2, 0, ARRIERE_LIDAR_DETECTER, 0, 0, 0, 0, 0, 0);
                // }
            }
            else
            {
                if (objetPresentArriere)
                {
                    Serial.printf("Fin objet derrière\n");
                    //                    sendCANMessage(LIDAR, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0);
                }
                objetPresentArriere = false;
            }
        }
    }

    // detectionRobots(odo_x_robot, odo_y_robot, theta_robot);
}

void initRPLidarA3M1(int frequence, int resolution, int vitesse)
{
    lidar.begin(Serial2);
    ledcSetup(PWM_CHANNEL, frequence, resolution); // Configuration du canal PWM_CHANNEL avec la fréquence et la résolution choisie
    ledcAttachPin(RPLIDAR_MOTOR, PWM_CHANNEL);     // Assigne le canal PWM au pin 18
    ledcWrite(PWM_CHANNEL, vitesse);               // Créer la tension en sortie choisie
    lidar.startScan();
}

// void detectionRobots(float odo_x_robot, float odo_y_robot, float theta_robot)
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

//       x = odo_x_robot + distance_lidar * cos(angle_lidar * DEG_TO_RAD);
//       y = odo_y_robot + distance_lidar * sin(angle_lidar * DEG_TO_RAD);

//       /*----------SI LES COORDONNES SONT DANS LE RECTANGLE----------*/

//       if (x >= DISTANCE_LIMITE && x <= RECTANGLE_X - DISTANCE_LIMITE && y >= DISTANCE_LIMITE && y <= RECTANGLE_Y - DISTANCE_LIMITE)
//       {
//         addOrUpdateRobot(x, y, 360 - angle_mesure, distance_lidar);
//       }
//     }
//   }
// }

void analyse(void)
{
    for (int i = 0; i < ANGLE_RESOLUTION; i++)
    {
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
//   // Serial.printf("Obstacle %d: x = %4.1f, y = %4.1f    Obstacle %d: x = %4.1f, y = %4.1f     Obstacle %d: x = %4.1f, y = %4.1f   ,  Notre Robot : x = %4.1f, y = %4.1f, theta = %4.1f\n", robots[0].id, robots[0].x, robots[0].y, robots[1].id, robots[1].x, robots[1].y, robots[2].id, robots[2].x, robots[2].y, odo_x_robot, odo_y_robot, theta_robot);
// }

// Timer
void init_Timer()
{
    static char idTimer = 0;     // le numéro du Timer de 0 à 3
    static int prescaler = 8000; // la valeur du diviseur de temps
    bool flag = true;            // vrai pour compter sur le front montant, faux pour compter sur le front descendant
                                 //  Configure le Prescaler a 80 le quartz de l ESP32 est cadence a 80Mhz => à vérifier pour l'esp32-32E, peut etre 40Mhz?
    // 80000000 / 80 = 1000000 tics / seconde
    timer = timerBegin(idTimer, prescaler, flag);
    timerAttachInterrupt(timer, &onTime, true); // fait qu'on execute la fonction onTime à chaque interruptions

    // Regle le declenchement d une alarme chaque seconde
    timerAlarmWrite(timer, 10, true); // freq de 250 000 Hz
    timerAlarmEnable(timer);          // active l'alarme
    Serial.println("Fin init Timer");
    mscount = 0;
    mscount2 = 0;
}
void onTime()
{ // fonction s'exécutent à chaque interruptions
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
