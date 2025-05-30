#include "ID_CAN.h"
#include "STRATEGIE.h"
#include "CAN_ESP32E.h"
#include "Variable.h"
uint8_t nbr_strat = 0;
extern volatile bool finMvt;

void setup_pin_strategie()
{

  pinMode(PIN_5V_FDC, OUTPUT);
  pinMode(PIN_READ_FDC, INPUT_PULLDOWN);
  digitalWrite(PIN_5V_FDC, true);

  // Pour choisir couleur de l'équipe
  pinMode(PIN_CHOIX_COULEUR, INPUT_PULLDOWN);
}

/*
void strategie_jaune_homologation()
{
  send_recalage(1225, 139, 90);
  delay(200);
  nbr_strat = 1;
  // uint16_t x_homologation[nbr_ps] = {1225, 750, 750, 300};
  // uint16_t y_homologation[nbr_ps] = {750, 750, 1500, 1800};

  uint16_t x_homologation[nbr_strat] = {1225};
  uint16_t y_homologation[nbr_strat] = {600};
  // uint16_t t_homologation[nbr_strat] = {-45};

  for (int i = 0; i < nbr_strat; i++)
  {
    Serial.println("message");
    uint16_t cmd_x = x_homologation[i];
    uint16_t cmd_y = y_homologation[i];

    uint8_t lowByte_x = cmd_x & 0xFF;         // Octet de poids faible
    uint8_t highByte_x = (cmd_x >> 8) & 0xFF; // Octet de poids fort

    uint8_t lowByte_y = cmd_y & 0xFF;         // Octet de poids faible
    uint8_t highByte_y = (cmd_y >> 8) & 0xFF; // Octet de poids fort

    sendCANMessage(POLAIRE, 0, 0, 8, i, highByte_x, lowByte_x, highByte_y, lowByte_y, nbr_strat, 0, 0);
    delay(10);
  }
}
*/

void attente(int max)
{
  int debut = millis();
  while ((millis() - debut) < max)
  {
    delay(1);
    if (finMvt)
    {
      finMvt = false;
      return;
    }
  }
}

void strategie_jaune_homologation()
{
  int debut = millis();

  send_recalage(1225, 139, 90);
  delay(200);
 
  // sendCANMessage(RECALAGE, 0, 0, 8, 2, 2, 0, 139, 0, 0, 0, 0);
  // delay(200);

  send_ligne_droite(-80);
  delay(1000);
  sendCANMessage(LACHER_ARRIERE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  delay(500);

  send_ligne_droite(400);
  attente(5000);
  delay(200);

  send_rotation(-45);
  attente(5000);
  delay(200);

  send_ligne_droite(900);
  attente(5000);
  delay(200);

  send_rotation(45);
  attente(5000);
  delay(200);

  while ((millis() - debut) < 92000)
  {
    delay(100);
  }

  send_ligne_droite(200);
  delay(2000);

  // nbr_strat = 1;
  // // uint16_t x_homologation[nbr_ps] = {1225, 750, 750, 300};
  // // uint16_t y_homologation[nbr_ps] = {750, 750, 1500, 1800};

  // uint16_t x_homologation[nbr_strat] = {400};
  // uint16_t y_homologation[nbr_strat] = {1600};
  // // uint16_t t_homologation[nbr_strat] = {-45};

  // for (int i = 0; i < nbr_strat; i++)
  // {
  //   Serial.println("message");
  //   uint16_t cmd_x = x_homologation[i];
  //   uint16_t cmd_y = y_homologation[i];

  //   uint8_t lowByte_x = cmd_x & 0xFF;         // Octet de poids faible
  //   uint8_t highByte_x = (cmd_x >> 8) & 0xFF; // Octet de poids fort

  //   uint8_t lowByte_y = cmd_y & 0xFF;         // Octet de poids faible
  //   uint8_t highByte_y = (cmd_y >> 8) & 0xFF; // Octet de poids fort

  //   sendCANMessage(POLAIRE, 0, 0, 8, i, highByte_x, lowByte_x, highByte_y, lowByte_y, nbr_strat, 0, 0);

  //   delay(20);
  // }
}

void strategie_bleu_homologation()
{
  int debut = millis();

  send_recalage(1775, 139, 90);
  delay(200);
  
  // sendCANMessage(RECALAGE, 0, 0, 8, 2, 2, 0, 139, 0, 0, 0, 0);
  delay(200);

  send_ligne_droite(-80);
  delay(1000);
  sendCANMessage(LACHER_ARRIERE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  delay(500);

  send_ligne_droite(400);
  attente(5000);
  delay(200);

  send_rotation(45);
  attente(5000);
  delay(200);

  send_ligne_droite(900);
  attente(5000);
  delay(200);

  send_rotation(-45);
  attente(5000);
  delay(200);

  while ((millis() - debut) < 92000)
  {
    delay(100);
  }

  send_ligne_droite(200);
  delay(2000);

  // nbr_strat = 1;

  // uint16_t x_homologation[nbr_strat] = {3000 - 400};
  // uint16_t y_homologation[nbr_strat] = {1600};

  // for (int i = 0; i < nbr_strat; i++)
  // {
  //   Serial.println("message");
  //   uint16_t cmd_x = x_homologation[i];
  //   uint16_t cmd_y = y_homologation[i];

  //   uint8_t lowByte_x = cmd_x & 0xFF;         // Octet de poids faible
  //   uint8_t highByte_x = (cmd_x >> 8) & 0xFF; // Octet de poids fort

  //   uint8_t lowByte_y = cmd_y & 0xFF;         // Octet de poids faible
  //   uint8_t highByte_y = (cmd_y >> 8) & 0xFF; // Octet de poids fort

  //   sendCANMessage(POLAIRE, 0, 0, 8, i, highByte_x, lowByte_x, highByte_y, lowByte_y, nbr_strat, 0, 0);
  //   delay(20);
  // }
}

void send_recalage(int x, int y, int theta)
{
  uint16_t cmd_x = x;
  uint16_t cmd_y = y;
  uint16_t cmd_theta = theta;

  uint8_t lowByte_x = cmd_x & 0xFF;         // Octet de poids faible
  uint8_t highByte_x = (cmd_x >> 8) & 0xFF; // Octet de poids fort

  uint8_t lowByte_y = cmd_y & 0xFF;                 // Octet de poids faible
  uint8_t highByte_y = (cmd_y >> 8) & 0xFF;         // Octet de poids fort
  uint8_t lowByte_theta = cmd_theta & 0xFF;         // Octet de poids faible
  uint8_t highByte_theta = (cmd_theta >> 8) & 0xFF; // Octet de poids fort

  delay(40);
  sendCANMessage(RECALAGE, 0, 0, 8, 0, 1, highByte_x, lowByte_x, 0, 0, 0, 0);
  delay(40);
  sendCANMessage(RECALAGE, 0, 0, 8, 0, 2, highByte_y, lowByte_y, 0, 0, 0, 0);
  delay(40);
  sendCANMessage(RECALAGE, 0, 0, 8, 0, 3, highByte_theta, lowByte_theta, 0, 0, 0, 0);
  delay(20);
}

void send_x_y_theta(int x, int y, int theta)
{

  nbr_strat = 1;
  for (int i = 0; i < nbr_strat; i++)
  {
    Serial.println("message");
    uint16_t cmd_x = x;
    uint16_t cmd_y = y;
    uint16_t cmd_theta = theta;

    uint8_t lowByte_x = cmd_x & 0xFF;         // Octet de poids faible
    uint8_t highByte_x = (cmd_x >> 8) & 0xFF; // Octet de poids fort

    uint8_t lowByte_y = cmd_y & 0xFF;                 // Octet de poids faible
    uint8_t highByte_y = (cmd_y >> 8) & 0xFF;         // Octet de poids fort
    uint8_t lowByte_theta = cmd_theta & 0xFF;         // Octet de poids faible
    uint8_t highByte_theta = (cmd_theta >> 8) & 0xFF; // Octet de poids fort

    sendCANMessage(POLAIRE, 0, 0, 8, i, highByte_x, lowByte_x, highByte_y, lowByte_y, nbr_strat, highByte_theta, lowByte_theta);
    delay(20);
  }
}

void send_ligne_droite(int cmd)
{

  Serial.println("LIGNE");

  uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
  uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort

  sendCANMessage(LIGNE_DROITE, 0, 0, 8, highByte, lowByte, 0x7B, 0, 0, 0, 0, 0);
}

void send_rotation(int cmd)
{

  Serial.println("Rotation ");

  uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
  uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort

  sendCANMessage(ROTATION, 0, 0, 8, highByte, lowByte, 0x7B, 0, 0, 0, 0, 0);
}
