#include <Arduino.h>

#include "M5_ANGLE8.h"
#include "UNIT_8ENCODER.h"
#include <M5Stack.h>
#include <SPI.h>
#include <mcp_can.h>

UNIT_8ENCODER encoder8;
M5_ANGLE8 angle8;

#define PIN_CAN_INT 15 // Set INT to pin 2
MCP_CAN can(12);
hw_timer_t *timer = NULL;

typedef union {
  byte data[8];

  struct {
    uint8_t idx;
    float value;
  } power;
  struct {
    uint16_t node_id;
    uint16_t type;
    uint32_t etc;
  } error_notify;

  struct {
    uint8_t cmd;
    uint8_t enable;
  } power_en;
  struct {
    uint8_t idx;
    float value;
  } set_protect_param;
  struct {
    int16_t delta_x;
    int16_t delta_y;
    uint16_t quality;
  } mouse;

  float speed;
  float voltage;
  float current;
  float temperature;
} can_data_t;

volatile static float real_speed[20], voltage[20], current[20], temperature[20];
struct {
  int16_t delta_x;
  int16_t delta_y;
  uint16_t quality;
} mouse;
struct {
  uint16_t node_id;
  uint16_t type;
  uint32_t etc;
} error_notify;
volatile int timer_cnt = 0, can_rx_cnt = 0;

void pollCanRx(void) {
  int32_t can_id;
  byte len;
  can_data_t rx;
  timer_cnt++;
  if (!digitalRead(PIN_CAN_INT)) {
    can_rx_cnt++;
    can.readMsgBuf(&can_id, &len, rx.data);
    return;
    if (can_id >= 0x200 && can_id < 0x210) {
      real_speed[can_id - 0x200] = rx.speed;
    }
    if (can_id >= 0x210 && can_id < 0x220) {
      voltage[can_id - 0x210] = rx.voltage;
    }
    if (can_id >= 0x220 && can_id < 0x230) {
      temperature[can_id - 0x220] = rx.temperature;
    }
    if (can_id >= 0x230 && can_id < 0x240) {
      current[can_id - 0x230] = rx.current;
    }
    if (can_id == 0x241) {
      mouse.delta_x = rx.mouse.delta_x;
      mouse.delta_y = rx.mouse.delta_y;
      mouse.quality = rx.mouse.quality;
    }
    if (can_id == 0) {
      error_notify.node_id = rx.error_notify.node_id;
      error_notify.type = rx.error_notify.type;
      error_notify.etc = rx.error_notify.etc;
    }
  }
}

void setup() {
  pinMode(G21, OUTPUT_OPEN_DRAIN);
  pinMode(G22, OUTPUT_OPEN_DRAIN);
  digitalWrite(G21, HIGH);
  digitalWrite(G22, HIGH);
  for (int i = 0; i < 20; i++) {
    digitalWrite(G22, LOW);
    delay(1);
    digitalWrite(G22, HIGH);
    delay(1);
  }

  M5.begin();
  M5.Speaker.begin();
  M5.Speaker.mute();
  Serial.begin(2000000);
  pinMode(PIN_CAN_INT, INPUT);
  if (can.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ)) {
    Serial.printf("fail init MCP2515!!\n");
    delay(100);
    M5.Power.reset();
  }
  can.setMode(MCP_NORMAL);

  if (!encoder8.begin(&Wire, ENCODER_ADDR, 21, 22, 200000UL)) {
    Serial.println("encoder8 Connect Error");
    delay(100);
    M5.Power.reset();
  }
  if (!angle8.begin(ANGLE8_I2C_ADDR)) {
    Serial.println("angle8 Connect Error");
    delay(100);
    M5.Power.reset();
  }
  Serial.printf("SSL-can tester start!!\n");

  for (int i = 0; i < 8; i++) {
    encoder8.resetCounter(i);
  }
  return;
  timer = timerBegin(0, 80, true);               // timer0,Peri-Clock:80MHz -> 1MHz count
  timerAttachInterrupt(timer, &pollCanRx, true); // set callback
  timerAlarmWrite(timer, 100, true);             // 100kHz pollint
  timerAlarmEnable(timer);
}

float limit(float input, float min, float max) {
  if (input > max) {
    return max;
  }
  if (input < min) {
    return min;
  }
  return input;
}

void loop() {
  float motor_speed[10]; // motor x4 + dribbler + servo
  bool power_enable = false, charge_enable = false;
  float charge_target_voltage = 100;
  float straght_kick_power = 0.1, chip_kick_power = 0.1;
  can_data_t tx;

  power_enable = encoder8.getSwitchStatus();
  charge_enable = angle8.getDigitalInput();
  tx.power_en.cmd = 0;
  tx.power_en.enable = power_enable;
  can.sendMsgBuf(0x010, 2, tx.data);
  if (power_enable) {
    encoder8.setLEDColor(8, 0x002000);
    Serial.printf("pw:EN ");
  } else {
    encoder8.setLEDColor(8, 0x001010);
    Serial.printf("pw:NO ");
  }

  tx.power_en.cmd = 1;
  tx.power_en.enable = charge_enable;
  can.sendMsgBuf(0x110, 2, tx.data);
  if (charge_enable) {
    angle8.setLEDColor(8, 0x002000, 255);
    Serial.printf("chg:EN ");
  } else {
    angle8.setLEDColor(8, 0x001010, 255);
    Serial.printf("chg:NO ");
  }

  /****************************************************************************/

  for (int i = 0; i < 5; i++) {
    motor_speed[i] = (float)encoder8.getEncoderValue(i); // 60rps / 1round

    motor_speed[i] = limit(motor_speed[i], -100, 100);
    if (motor_speed[i] > 0) {
      encoder8.setLEDColor(i, 0x001000);
    } else if (motor_speed[i] < 0) {
      encoder8.setLEDColor(i, 0x100000);
    } else {
      encoder8.setLEDColor(i, 0x001010);
    }
    if (!encoder8.getButtonStatus(i)) {
      encoder8.resetCounter(i);
    }
  }

  // servo
  motor_speed[5] = (float)encoder8.getEncoderValue(5) / 60; // 60cnt / 1round
  motor_speed[5] = limit(motor_speed[5], -1, 1);
  if (!encoder8.getButtonStatus(5)) {
    encoder8.resetCounter(5);
  }
  if (motor_speed[5] > 0) {
    encoder8.setLEDColor(5, 0x001000);
  } else if (motor_speed[5] < 0) {
    encoder8.setLEDColor(5, 0x100000);
  } else {
    encoder8.setLEDColor(5, 0x001010);
  }

  for (int i = 0; i < 5; i++) {
    tx.speed = motor_speed[i];
    can.sendMsgBuf(0x100 + i, 4, tx.data);
    Serial.printf("M%d : %+4.0f / ", i, motor_speed[i]);
  }

  tx.speed = motor_speed[5];
  Serial.printf("SV %+2.1f %d ", motor_speed[5], can.sendMsgBuf(0x105, 4, tx.data));

  /****************************************************************************/

  charge_target_voltage = (float)angle8.getAnalogInput(0, _8bit) * 450 / 255;
  straght_kick_power = (float)angle8.getAnalogInput(1, _8bit) / 255;
  chip_kick_power = (float)angle8.getAnalogInput(2, _8bit) / 255;
  charge_target_voltage = limit(charge_target_voltage, 20, 450);
  straght_kick_power = limit(straght_kick_power, 0.1, 1.0);
  chip_kick_power = limit(chip_kick_power, 0.1, 1.0);

  tx.power.idx = 0; // charge voltage
  tx.power.value = charge_target_voltage;
  can.sendMsgBuf(0x110, 8, tx.data);
  if (charge_target_voltage < 100) {
    angle8.setLEDColor(0, 0x000010, 255);
  } else if (charge_target_voltage < 300) {
    angle8.setLEDColor(0, 0x101000, 255);
  } else {
    angle8.setLEDColor(0, 0x100000, 255);
  }

  if (straght_kick_power < 0.2) {
    angle8.setLEDColor(1, 0x000010, 255);
  } else if (straght_kick_power < 0.9) {
    angle8.setLEDColor(1, 0x101000, 255);
  } else {
    angle8.setLEDColor(1, 0x100000, 255);
  }

  if (chip_kick_power < 0.2) {
    angle8.setLEDColor(2, 0x000010, 255);
  } else if (chip_kick_power < 0.9) {
    angle8.setLEDColor(2, 0x101000, 255);
  } else {
    angle8.setLEDColor(2, 0x100000, 255);
  }

  if (voltage[5] * 0.9 > charge_target_voltage) {
    encoder8.setLEDColor(6, 0x001000);
    encoder8.setLEDColor(7, 0x001000);
  } else {
    encoder8.setLEDColor(6, 0x100000);
    encoder8.setLEDColor(7, 0x100000);
  }
  Serial.printf("HV : %3.0f strt %2.1f chip %2.1f", charge_target_voltage, straght_kick_power, chip_kick_power);

  if (!encoder8.getButtonStatus(6)) {
    tx.data[0] = 2; // kicker select
    tx.data[1] = 0;
    can.sendMsgBuf(0x110, 2, tx.data);

    tx.data[0] = 3; // kick start, power
    tx.data[1] = 255 * straght_kick_power;
    can.sendMsgBuf(0x110, 2, tx.data);
    Serial.printf("\nKICK!!! (strt)\n");
  }
  if (!encoder8.getButtonStatus(7)) {

    tx.data[0] = 2; // kicker select
    tx.data[1] = 1;
    can.sendMsgBuf(0x110, 2, tx.data);

    tx.data[0] = 3; // kick start, power
    tx.data[1] = 255 * chip_kick_power;
    can.sendMsgBuf(0x110, 2, tx.data);
    Serial.printf("\nKICK!!! (chip)\n");
  }

  for (int i = 0; i < 5; i++) {
    Serial.printf("sped %+4.0f ", real_speed[i]);
  }
  Serial.printf("BattV %3.1f,I %+2.0f, Boost %3.0f ", voltage[4], current[4], voltage[5]);
  Serial.printf("MS X%+4d Y %+4d Q %3d ", mouse.delta_x, mouse.delta_y, mouse.quality);
  Serial.printf("temp FET %2d coil %2d %2d ", temperature[4], temperature[5], temperature[6]);
  Serial.printf("error node %3d type %3d etc %8d / %+5.2f ", error_notify.node_id, error_notify.type, error_notify.etc, (float)error_notify.etc);
  Serial.printf("cnt %6d\n", can_rx_cnt);
  timer_cnt = 0;
  can_rx_cnt = 0;
}
