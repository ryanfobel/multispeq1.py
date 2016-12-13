#ifndef ___NODE__H___
#define ___NODE__H___

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <Arduino.h>
#include <NadaMQ.h>
#include <CArrayDefs.h>
#include "RPCBuffer.h"  // Define packet sizes
#include "Multispeq1/Properties.h"  // Define package name, URL, etc.
#include "Multispeq1/CommandProcessor.h"
#include <BaseNodeRpc/BaseNode.h>
#include <BaseNodeRpc/BaseNodeSerialHandler.h>
#include <BaseNodeRpc/SerialHandler.h>
#include <Multispeq1/aligned_alloc.h>
#include <LinkedList.h>
#include "PAR.h"

namespace multispeq1 {
  
const size_t FRAME_SIZE = (3 * sizeof(uint8_t)  // Frame boundary
                           - sizeof(uint16_t)  // UUID
                           - sizeof(uint16_t)  // Payload length
                           - sizeof(uint16_t));  // CRC

class Node;

class Node :
  public BaseNode,
  public BaseNodeSerialHandler {
public:
  typedef PacketParser<FixedPacket> parser_t;

  static const uint32_t BUFFER_SIZE = 8192;  // >= longest property string

  uint8_t buffer_[BUFFER_SIZE];

  Node() : BaseNode() {
    pinMode(LED_BUILTIN, OUTPUT);
  }

  void begin();
  UInt8Array get_buffer() {
    /* This is a required method to provide a temporary buffer to the
     * `BaseNode...` classes. */
    return UInt8Array_init(sizeof(buffer_), buffer_);
  }
  /****************************************************************************
   * # User-defined methods #
   *
   * Add new methods below.  When Python package is generated using the
   * command, `paver sdist` from the project root directory, the signatures of
   * the methods below will be scanned and code will automatically be generated
   * to support calling the methods from Python over a serial connection.
   *
   * e.g.
   *
   *     bool less_than(float a, float b) { return a < b; }
   *
   * See [`arduino_rpc`][1] and [`base_node_rpc`][2] for more details.
   *
   * [1]: https://github.com/wheeler-microfluidics/arduino_rpc
   * [2]: https://github.com/wheeler-microfluidics/base_node_rpc
   */

  // ##########################################################################

  int get_light_intensity(int _averages) { return PAR::get_light_intensity(_averages); }
};

}  // namespace multispeq1

extern multispeq1::Node node_obj;
extern multispeq1::CommandProcessor<multispeq1::Node> command_processor;

#endif  // #ifndef ___NODE__H___
