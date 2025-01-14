#include "vaillantx6.h"

void logCmd(const char *tag, byte *cmd) {
  ESP_LOGD("vaillantx6", "%s: 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x", tag, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]);
}

uint8_t VaillantReturnTypeLength(VaillantReturnTypes t) {
  switch (t) {
    case SensorState:
    case Bool:
    case Minutes:
      return 1;
    case Temperature:
      return 2;
    default:
      return 0;
  }
}

float VaillantParseTemperature(byte *answerBuff, uint8_t offset) {
  int16_t i = (answerBuff[offset] << 8) | answerBuff[offset + 1];
  return i / (16.0f);
}

int VaillantParseBool(byte *answerBuff, uint8_t offset) {
  switch (answerBuff[offset]) {
    case 0xF0:
    case 0x00:
      return 0;
    case 0x0F:
    case 0x01:
      return 1;
    default:
      ESP_LOGE("VaillantParseBool", "Unable to parse a bool from 0x%.2x", answerBuff[offset]);
      return -1;
  }
}

const VaillantCommand vaillantCommands[] = {
    {"Vorlauf Ist", 0x18, Temperature},
    {"Aussen Temperatur", 0x6A, Temperature},
    {"Vorlauf Soll", 0x39, Temperature},
    {"Vorlauf 789 Soll", 0x25, Temperature},
    {"Rücklauf Ist", 0x98, Temperature},
    {"Speicher Ist", 0x17, Temperature},
    {"Speicher Soll", 0x04, Temperature},
    {"Brauchwasser Ist", 0x16, Temperature},
    {"Brauchwasser Soll", 0x01, Temperature},
    {"Drehzahl Soll", 0x24, Temperature},
    {"Drehzahl Ist", 0x83, Temperature},
    {"Brenner", 0x0d, Bool},
    {"Winter", 0x08, Bool},
    {"Pumpe", 0x44, Bool},
    {"Flammsignal", 0x05, Bool},
    {"Gasmagnetventil", 0x48, Bool},
    {"Zünder", 0x49, Bool},
    {"Verbliebene Brennsperrzeit", 0x38, Minutes},
};

const byte vaillantCommandsSize = sizeof(vaillantCommands) / sizeof *(vaillantCommands);

Vaillantx6Sensor::Vaillantx6Sensor(UARTComponent *parent, const VaillantCommand *command, uint32_t update_interval) 
    : PollingComponent(update_interval), UARTDevice(parent), command_(command) {}

void Vaillantx6Sensor::set_sensor(Sensor *sensor) {
  sensor_ = sensor;
}

void Vaillantx6Sensor::set_binary_sensor(BinarySensor *binary_sensor) {
  binary_sensor_ = binary_sensor;
}

byte Vaillantx6Sensor::checksum(byte *data, byte len) {
  byte checksum = 0;
  for (byte i = 0; i < len; i++) {
    if (checksum & 0x80) {
      checksum = (checksum << 1) | 1;
      checksum = checksum ^ 0x18;
    } else {
      checksum = checksum << 1;
    }
    checksum = checksum ^ data[i];
  }
  return checksum;
}

bool Vaillantx6Sensor::checksumOk(byte *answerBuff, byte len) {
  logCmd("Answer", answerBuff);
  return checksum(answerBuff, len - 1) == answerBuff[len - 1];
}

int Vaillantx6Sensor::buildPacket(byte *packet, byte address) {
  const byte startBytes[4] = {0x07, 0x00, 0x00, 0x00};
  int i = 0;
  while (i < sizeof(startBytes)) {
    packet[i] = startBytes[i];
    i++;
  }
  packet[i] = address;
  i++;
  packet[i] = 0x00;
  i++;
  packet[i] = checksum(packet, 6);
  return i;
}

int Vaillantx6Sensor::sendPacket(byte *answerBuff, byte *packet) {
  int answerLen = 0;
  int readRetry = 3;
  write_array(packet, CMD_LENGTH);

  while (available() < 1) {
    delay(50);
    readRetry--;
    if (readRetry < 0) {
      ESP_LOGE("Vaillantx6 sendPacket", "Timed out waiting for bytes from Vaillant");
      return -3;
    }
  }
  answerLen = peek();

  if (answerLen > ANSWER_LENGTH) {
    ESP_LOGE("Vaillantx6 sendPacket", "Received an answer of unexpected length %d, ignoring", answerLen);
    while (available()) {
      read();
    }
    return -1;
  }
  read_array(answerBuff, answerLen);
  if (!checksumOk(answerBuff, answerLen)) {
    ESP_LOGE("Vaillantx6 sendPacket", "Packet has invalid checksum");
    while (available()) {
      read();
    }
    return -2;
  }
  return answerLen;
}

void Vaillantx6Sensor::update() {
  byte *cmdPacket = (byte *)malloc(sizeof(byte) * CMD_LENGTH);
  byte *answerBuff = (byte *)malloc(sizeof(byte) * ANSWER_LENGTH);

  buildPacket(cmdPacket, command_->Address);
  logCmd(command_->Name.c_str(), cmdPacket);

  int answerLen = sendPacket(answerBuff, cmdPacket);
  if (answerLen < 0) {
    ESP_LOGE("Vaillantx6", "sendPacket returned an error: %d", answerLen);
  } else if (answerLen > 3) {
    switch (command_->ReturnType) {
      case Temperature:
        if (sensor_ != nullptr) {
          float temp = VaillantParseTemperature(answerBuff, 2);
          sensor_->publish_state(temp);
        }
        break;
      case Bool:
        if (binary_sensor_ != nullptr) {
          int b = VaillantParseBool(answerBuff, 2);
          if (b >= 0) {
            binary_sensor_->publish_state(b);
          }
        }
        break;
      case Minutes:
        if (sensor_ != nullptr) {
          sensor_->publish_state(answerBuff[2]);
        }
        break;
      default:
        break;
    }
  }

  free(cmdPacket);
  free(answerBuff);
}