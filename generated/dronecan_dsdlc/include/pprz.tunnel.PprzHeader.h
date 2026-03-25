#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define PPRZ_TUNNEL_PPRZHEADER_MAX_SIZE 4
#define PPRZ_TUNNEL_PPRZHEADER_SIGNATURE (0x4B786B5729E917EFULL)


struct pprz_tunnel_PprzHeader {
    uint8_t source;
    uint8_t destination;
    uint8_t classId;
    uint8_t componentId;
    uint8_t msgId;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t pprz_tunnel_PprzHeader_encode(struct pprz_tunnel_PprzHeader* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool pprz_tunnel_PprzHeader_decode(const CanardRxTransfer* transfer, struct pprz_tunnel_PprzHeader* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _pprz_tunnel_PprzHeader_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_PprzHeader* msg, bool tao);
static inline bool _pprz_tunnel_PprzHeader_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_PprzHeader* msg, bool tao);
void _pprz_tunnel_PprzHeader_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_PprzHeader* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->source);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->destination);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 4, &msg->classId);
    *bit_ofs += 4;
    canardEncodeScalar(buffer, *bit_ofs, 4, &msg->componentId);
    *bit_ofs += 4;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->msgId);
    *bit_ofs += 8;
}

/*
 decode pprz_tunnel_PprzHeader, return true on failure, false on success
*/
bool _pprz_tunnel_PprzHeader_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_PprzHeader* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->source);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->destination);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->classId);
    *bit_ofs += 4;

    canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->componentId);
    *bit_ofs += 4;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->msgId);
    *bit_ofs += 8;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct pprz_tunnel_PprzHeader sample_pprz_tunnel_PprzHeader_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
