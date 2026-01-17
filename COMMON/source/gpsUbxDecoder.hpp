/**
 * @file gpsUbxDecoder.hpp
 * @brief Streaming decoder for UBX NAV messages.
 */
#pragma once

#include <concepts>
#include <type_traits>
#include <cstdint>
#include <cstddef>
#include <span>
#include <array>
#include <etl/vector.h>
#include "etl/span.h"
#include "ch.h"
//#include "stdutil.h"
#include "gpsUbxMessageDef.hpp"





/*
  classe qui doit decoder les messages UBX PVT, DOP et SAT 

  * il lui faut :
  ° un constructeur qui prend les 3 callback en paramètres pour chacun des 3 messages
  ° un buffer d'accumulation  etl::vector<uint8_t, 1024>;
  ° une methode feed(etl::span<uint8_t> in) qui append le buffer en paramètre
  et appelle la macchine à état pour le decodage de message UBX
  ° Quand un message est decodé en entier on appelle une callback.


Offset  Taille    Nom
0       1         SYNC1 = 0xB5
1       1         SYNC2 = 0x62
2       1         CLASS
3       1         ID
4       2         LENGTH (LSB, MSB) = taille du payload
6       N         PAYLOAD
6+N     1         CK_A
7+N     1         CK_B




*/

namespace UBX {
  /**
   * @brief Callback set for UBX NAV messages handled by the decoder.
   */
  struct DecoderConf {
    bool (*navPvtCb)(const NavPvt& msg);
    bool (*navDopCb)(const NavDop& msg);
    bool (*navSatCb)(const NavSat& msg);
  };
  
  /**
   * @brief Incremental UBX decoder that validates checksums and dispatches NAV messages.
   */
  class Decoder {
    enum State {WAIT_FOR_SYNC1, WAIT_FOR_SYNC2, WAIT_FOR_CLASS, WAIT_FOR_ID,
		WAIT_FOR_LEN1, WAIT_FOR_LEN2,
		PROCESSING_PAYLOAD, WAIT_FOR_CHECKSUM_1, WAIT_FOR_CHECKSUM_2};
  public:
    /** @brief UBX sync bytes. */
    static constexpr uint8_t Sync1 = 0xB5;
    static constexpr uint8_t Sync2 = 0x62;

    /**
     * @brief Construct the decoder with a callback configuration.
     */
    Decoder(const DecoderConf& _cfg) : cfg(_cfg) {}
    /**
     * @brief Feed a span of raw bytes into the state machine.
     */
    void feed(etl::span<const uint8_t> data);
    
  private:
    /** @brief Reset the parser to the initial sync state. */
    void reset();
    /** @brief Update UBX checksum with a single byte. */
    inline void updateChecksum(uint8_t byte);
    /** @brief Dispatch a fully decoded payload to its callback. */
    bool dispatch();

    etl::vector<uint8_t, 1024> payload;
    const DecoderConf& cfg;

    uint16_t expectedLength = 0;
    uint8_t cls = 0;
    uint8_t id = 0;
    
    uint8_t ckA = 0;
    uint8_t ckB = 0;
    uint8_t recvCkA = 0;
    uint8_t recvCkB = 0;
    State state = WAIT_FOR_SYNC1;
  };
  
}
