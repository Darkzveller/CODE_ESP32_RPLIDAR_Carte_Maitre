#include <Arduino.h>

#ifndef Variable_H
#define Variable_H
// Déclaration des variables externes

//***********Stratégie******************* */
#define PIN_CHOIX_COULEUR 32
#define PIN_5V_FDC 25
#define PIN_READ_FDC 33
#define TIME_TO_SEND_DATA_CAN 20
#define HOMOLOGATION
// #define UN_SPOT
extern int vitesse ;
extern bool start_match ;
extern int etat_match ;


// Parametre FreeRTOS
#define Te 10
#define Tcan 10

//***********Variable pour détection lidar******************* */
#define AVANT_LIDAR_DETECTER 1
#define ARRIERE_LIDAR_DETECTER 0

//***********LOOP******************* */

extern bool flag_controle;
//***********Parametre relever du robot******************* */

 extern int16_t odo_x_robot;
 extern int16_t odo_y_robot;
 extern int16_t theta_robot;

//***********CAN******************* */

#define SIZE_FIFO 32

typedef struct CANMessage
{
  bool extd = false;
  bool rtr = false;
  uint32_t id = 0;
  uint8_t lenght = 0;
  uint8_t data[8] = {0};
} CANMessage;
extern CANMessage rxMsg; // data received by CAN to control the robot
extern int etat_lecture_can;
extern unsigned char FIFO_ecriture;
extern signed char FIFO_lecture;
extern signed char FIFO_occupation;
extern signed char FIFO_max_occupation;
extern bool flag_can_printf;

//***********Ordre de déplacement******************* */
#define TYPE_DEPLACEMENT_IMMOBILE 1
#define TYPE_DEPLACEMENT_LIGNE_DROITE 2
#define TYPE_DEPLACEMENT_ROTATION 3
#define TYPE_DEPLACEMENT_X_Y_THETA 4
#define TYPE_DEPLACEMENT_RECALAGE 5
#define TYPE_VIDE 6
#define TYPE_DEPLACEMENT_X_Y_POLAIRE 7 
#define TYPE_EVITEMENT 8

extern bool flag_fin_mvt;

extern bool stop_start_robot_fin_match;
#endif
