#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define PPRZ_TUNNEL_XBEEHEADERRX_MAX_SIZE 5
#define PPRZ_TUNNEL_XBEEHEADERRX_SIGNATURE (0xF5FEB9317FB22BFCULL)


struct pprz_tunnel_XbeeHeaderRx {
    uint8_t role;
    uint16_t src_id;
    uint8_t rssi;
    uint8_t options;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t pprz_tunnel_XbeeHeaderRx_encode(struct pprz_tunnel_XbeeHeaderRx* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool pprz_tunnel_XbeeHeaderRx_decode(const CanardRxTransfer* transfer, struct pprz_tunnel_XbeeHeaderRx* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _pprz_tunnel_XbeeHeaderRx_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_XbeeHeaderRx* msg, bool tao);
static inline bool _pprz_tunnel_XbeeHeaderRx_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_XbeeHeaderRx* msg, bool tao);
void _pprz_tunnel_XbeeHeaderRx_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_XbeeHeaderRx* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->role);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->src_id);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->rssi);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->options);
    *bit_ofs += 8;
}

/*
 decode pprz_tunnel_XbeeHeaderRx, return true on failure, false on success
*/
bool _pprz_tunnel_XbeeHeaderRx_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_XbeeHeaderRx* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->role);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->src_id);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->rssi);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->options);
    *bit_ofs += 8;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct pprz_tunnel_XbeeHeaderRx sample_pprz_tunnel_XbeeHeaderRx_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
