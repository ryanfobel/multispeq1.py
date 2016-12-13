#include "Arduino.h"
#include "EEPROM.h"
#include "SPI.h"
#include "Wire.h"
#include "LinkedList.h"
#include "Memory.h"  // Required replacing memory functions with stubs returning 0.
#include "ArduinoRpc.h"
#include "nanopb.h"
#include "NadaMQ.h"  // Required replacing `#ifndef AVR` with `#if !defined(AVR) && !defined(__arm__)`
#include "CArrayDefs.h"
#include "RPCBuffer.h"
#include "BaseNodeRpc.h"  // Check for changes (may have removed some include statements...
#include "Photosynq.h"
#include "NodeCommandProcessor.h"
#include "ADC.h"
#include "Node.h"


photosynq::Node node_obj;
photosynq::CommandProcessor<photosynq::Node> command_processor(node_obj);

// when the measurement finishes, this will be called
// first: see which pin finished and then save the measurement into the correct buffer
void adc0_isr() {
  node_obj.on_adc_done();
  //ADC0_RA; // clear interrupt
}

void serialEvent() { node_obj.serial_handler_.receiver()(Serial.available()); }


void setup() {
  node_obj.begin();
}


void loop() {
#ifndef DISABLE_SERIAL
  /* Parse all new bytes that are available.  If the parsed bytes result in a
   * completed packet, pass the complete packet to the command-processor to
   * process the request. */
  if (node_obj.serial_handler_.packet_ready()) {
    node_obj.serial_handler_.process_packet(command_processor);
  }
#endif  // #ifndef DISABLE_SERIAL
  node_obj.loop();
}

void dma_ch0_isr(void) {
  DMA_CINT = 0;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 0;
}
void dma_ch1_isr(void) {
  DMA_CINT = 1;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 1;
}
void dma_ch2_isr(void) {
  DMA_CINT = 2;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 2;
}
void dma_ch3_isr(void) {
  DMA_CINT = 3;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 3;
}
void dma_ch4_isr(void) {
  DMA_CINT = 4;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 4;
}
void dma_ch5_isr(void) {
  DMA_CINT = 5;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 5;
}
void dma_ch6_isr(void) {
  DMA_CINT = 6;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 6;
}
void dma_ch7_isr(void) {
  DMA_CINT = 7;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 7;
}
void dma_ch8_isr(void) {
  DMA_CINT = 8;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 8;
}
void dma_ch9_isr(void) {
  DMA_CINT = 9;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 9;
}
void dma_ch10_isr(void) {
  DMA_CINT = 10;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 10;
}
void dma_ch11_isr(void) {
  DMA_CINT = 11;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 11;
}
void dma_ch12_isr(void) {
  DMA_CINT = 12;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 12;
}
void dma_ch13_isr(void) {
  DMA_CINT = 13;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 13;
}
void dma_ch14_isr(void) {
  DMA_CINT = 14;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 14;
}
void dma_ch15_isr(void) {
  DMA_CINT = 15;
  PDB0_SC = 0;  // Stop PDB timer.
  node_obj.dma_channel_done_ = 15;
}
