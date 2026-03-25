#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <pprz.tunnel.PprzHeader.h>
#include <pprz.tunnel.XbeeHeader.h>


#define PPRZ_TUNNEL_TELEMETRY_MAX_SIZE 267
#define PPRZ_TUNNEL_TELEMETRY_SIGNATURE (0x65EB80AB804C9B18ULL)
#define PPRZ_TUNNEL_TELEMETRY_ID 22101

#define PPRZ_TUNNEL_TELEMETRY_UPLINK 0
#define PPRZ_TUNNEL_TELEMETRY_DOWNLINK 1

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class pprz_tunnel_Telemetry_cxx_iface;
#endif

struct pprz_tunnel_Telemetry {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = pprz_tunnel_Telemetry_cxx_iface;
#endif
    bool direction;
    struct pprz_tunnel_XbeeHeader xbeeHeader;
    struct pprz_tunnel_PprzHeader pprzHeader;
    struct { uint16_t len; uint8_t data[256]; }payload;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t pprz_tunnel_Telemetry_encode(struct pprz_tunnel_Telemetry* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool pprz_tunnel_Telemetry_decode(const CanardRxTransfer* transfer, struct pprz_tunnel_Telemetry* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _pprz_tunnel_Telemetry_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_Telemetry* msg, bool tao);
static inline bool _pprz_tunnel_Telemetry_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_Telemetry* msg, bool tao);
void _pprz_tunnel_Telemetry_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_Telemetry* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 1, &msg->direction);
    *bit_ofs += 1;
    _pprz_tunnel_XbeeHeader_encode(buffer, bit_ofs, &msg->xbeeHeader, false);
    _pprz_tunnel_PprzHeader_encode(buffer, bit_ofs, &msg->pprzHeader, false);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint16_t payload_len = msg->payload.len > 256 ? 256 : msg->payload.len;
#pragma GCC diagnostic pop
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 9, &payload_len);
        *bit_ofs += 9;
    }
    for (size_t i=0; i < payload_len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->payload.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode pprz_tunnel_Telemetry, return true on failure, false on success
*/
bool _pprz_tunnel_Telemetry_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_Telemetry* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->direction);
    *bit_ofs += 1;

    if (_pprz_tunnel_XbeeHeader_decode(transfer, bit_ofs, &msg->xbeeHeader, false)) {return true;}

    if (_pprz_tunnel_PprzHeader_decode(transfer, bit_ofs, &msg->pprzHeader, false)) {return true;}

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 9, false, &msg->payload.len);
        *bit_ofs += 9;
    } else {
        msg->payload.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->payload.len > 256) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->payload.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->payload.data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct pprz_tunnel_Telemetry sample_pprz_tunnel_Telemetry_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
BROADCAST_MESSAGE_CXX_IFACE(pprz_tunnel_Telemetry, PPRZ_TUNNEL_TELEMETRY_ID, PPRZ_TUNNEL_TELEMETRY_SIGNATURE, PPRZ_TUNNEL_TELEMETRY_MAX_SIZE);
#endif
#endif
