#ifndef VAILLANTX6_H
#define VAILLANTX6_H

#include "esphome.h"
#include <string>

#define CMD_LENGTH 7
#define ANSWER_LENGTH 8
#define RETURN_TYPE_COUNT 3

typedef unsigned char byte;

void logCmd(const char *tag, byte *cmd);

enum VaillantReturnTypes {
  None = 0,
  Temperature,
  SensorState,
  Bool,
  Minutes
};

uint8_t VaillantReturnTypeLength(VaillantReturnTypes t);

float VaillantParseTemperature(byte *answerBuff, uint8_t offset);

int VaillantParseBool(byte *answerBuff, uint8_t offset);

struct VaillantCommand {
  std::string Name;
  byte Address;
  VaillantReturnTypes ReturnType;
};

extern const VaillantCommand vaillantCommands[];
extern const byte vaillantCommandsSize;

class Vaillantx6Sensor : public PollingComponent, public UARTDevice {
 public:
  Sensor *sensor_{nullptr};
  BinarySensor *binary_sensor_{nullptr};
  const VaillantCommand *command_;

  Vaillantx6Sensor(UARTComponent *parent, const VaillantCommand *command, uint32_t update_interval);
  void set_sensor(Sensor *sensor);
  void set_binary_sensor(BinarySensor *binary_sensor);
  byte checksum(byte *data, byte len);
  bool checksumOk(byte *answerBuff, byte len);
  int buildPacket(byte *packet, byte address);
  int sendPacket(byte *answerBuff, byte *packet);
  void update() override;
};

#endif  // VAILLANTX6SENSOR_H