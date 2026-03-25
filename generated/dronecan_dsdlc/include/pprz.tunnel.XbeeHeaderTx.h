#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define PPRZ_TUNNEL_XBEEHEADERTX_MAX_SIZE 5
#define PPRZ_TUNNEL_XBEEHEADERTX_SIGNATURE (0x62B4C52A6D6B58F2ULL)


struct pprz_tunnel_XbeeHeaderTx {
    uint8_t role;
    uint8_t frame_id;
    uint16_t dest_id;
    uint8_t options;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t pprz_tunnel_XbeeHeaderTx_encode(struct pprz_tunnel_XbeeHeaderTx* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool pprz_tunnel_XbeeHeaderTx_decode(const CanardRxTransfer* transfer, struct pprz_tunnel_XbeeHeaderTx* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _pprz_tunnel_XbeeHeaderTx_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_XbeeHeaderTx* msg, bool tao);
static inline bool _pprz_tunnel_XbeeHeaderTx_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_XbeeHeaderTx* msg, bool tao);
void _pprz_tunnel_XbeeHeaderTx_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_XbeeHeaderTx* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->role);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->frame_id);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->dest_id);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->options);
    *bit_ofs += 8;
}

/*
 decode pprz_tunnel_XbeeHeaderTx, return true on failure, false on success
*/
bool _pprz_tunnel_XbeeHeaderTx_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_XbeeHeaderTx* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->role);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->frame_id);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->dest_id);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->options);
    *bit_ofs += 8;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct pprz_tunnel_XbeeHeaderTx sample_pprz_tunnel_XbeeHeaderTx_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
