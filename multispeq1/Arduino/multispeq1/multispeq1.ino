#include "Arduino.h"
#include "EEPROM.h"
#include "SPI.h"
#include "LinkedList.h"
#include "Memory.h"  // Required replacing memory functions with stubs returning 0.
#include "ArduinoRpc.h"
#include "nanopb.h"
#include "NadaMQ.h"  // Required replacing `#ifndef AVR` with `#if !defined(AVR) && !defined(__arm__)`
#include "CArrayDefs.h"
#include "RPCBuffer.h"
#include "BaseNodeRpc.h"  // Check for changes (may have removed some include statements...
#include "Multispeq1.h"
#include "NodeCommandProcessor.h"
#include "Node.h"

// includes
#include "defines.h"
#include <i2c_t3.h>
#include <Time.h>                   // enable real time clock library
#define EXTERN
#include "eeprom.h"
#include "serial.h"
//#include <SPI.h>
#include "util.h"
#include <TimeLib.h>

multispeq1::Node node_obj;
multispeq1::CommandProcessor<multispeq1::Node> command_processor(node_obj);

void setup() {
  node_obj.begin();
}  // setup() - now execute loop()

void setup_pins();
void unset_pins();
void readSpectrometer(int intTime, int delay_time, int read_time, int accumulateMode);
void print_data();

