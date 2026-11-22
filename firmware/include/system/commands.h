#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>

// USB serial command line. Type "help" in the PlatformIO serial monitor.
// Non-blocking: call poll() from loop().
namespace commands {

void poll();
void printHelp(Print& out);

}  // namespace commands

#endif  // COMMANDS_H
