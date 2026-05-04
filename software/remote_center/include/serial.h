#ifndef SERIAL_H
#define SERIAL_H

#include <SoftwareSerial.h>

void initSoftwareSerial();
void sendCommands(String command);

#endif // SERIAL_H