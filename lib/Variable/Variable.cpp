#include "Variable.h"
//***********Stratégie******************* */
#define PIN_CHOIX_COULEUR 32
#define PIN_5V_FDC 25
#define PIN_READ_FDC 33
#define TIME_TO_SEND_DATA_CAN 20
#define HOMOLOGATION
// #define UN_SPOT
int vitesse = 90;
bool start_match = 0;
int etat_match = 0;

//***********Loop******************* */

bool flag_controle = false;

//***********Parametre relever du robot******************* */

 int16_t odo_x_robot=0;
 int16_t odo_y_robot=0;
 int16_t theta_robot=0;
//***********CAN******************* */

CANMessage rxMsg;
int etat_lecture_can;
unsigned char FIFO_ecriture = 0;
signed char FIFO_lecture = 0;
signed char FIFO_occupation = 0;
signed char FIFO_max_occupation = 0;
bool flag_can_printf = false;
//***********Ordre de déplacement******************* */
bool flag_fin_mvt = true;
bool stop_start_robot_fin_match = false;

//*********** Qu'est ce qu'on voit au borne de la batterie******************* */
float courant = 0;
float tension = 0;
float puissance = 0;