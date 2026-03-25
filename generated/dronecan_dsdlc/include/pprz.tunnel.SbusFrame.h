#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define PPRZ_TUNNEL_SBUSFRAME_MAX_SIZE 24
#define PPRZ_TUNNEL_SBUSFRAME_SIGNATURE (0xF6E542200C92AC53ULL)
#define PPRZ_TUNNEL_SBUSFRAME_ID 22100

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class pprz_tunnel_SbusFrame_cxx_iface;
#endif

struct pprz_tunnel_SbusFrame {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = pprz_tunnel_SbusFrame_cxx_iface;
#endif
    uint8_t flags;
    struct { uint8_t len; uint16_t data[16]; }channels;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t pprz_tunnel_SbusFrame_encode(struct pprz_tunnel_SbusFrame* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool pprz_tunnel_SbusFrame_decode(const CanardRxTransfer* transfer, struct pprz_tunnel_SbusFrame* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _pprz_tunnel_SbusFrame_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_SbusFrame* msg, bool tao);
static inline bool _pprz_tunnel_SbusFrame_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_SbusFrame* msg, bool tao);
void _pprz_tunnel_SbusFrame_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_SbusFrame* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->flags);
    *bit_ofs += 8;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t channels_len = msg->channels.len > 16 ? 16 : msg->channels.len;
#pragma GCC diagnostic pop
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 5, &channels_len);
        *bit_ofs += 5;
    }
    for (size_t i=0; i < channels_len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 11, &msg->channels.data[i]);
        *bit_ofs += 11;
    }
}

/*
 decode pprz_tunnel_SbusFrame, return true on failure, false on success
*/
bool _pprz_tunnel_SbusFrame_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_SbusFrame* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->flags);
    *bit_ofs += 8;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->channels.len);
        *bit_ofs += 5;
    } else {
        msg->channels.len = ((transfer->payload_len*8)-*bit_ofs)/11;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->channels.len > 16) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->channels.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 11, false, &msg->channels.data[i]);
        *bit_ofs += 11;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct pprz_tunnel_SbusFrame sample_pprz_tunnel_SbusFrame_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
BROADCAST_MESSAGE_CXX_IFACE(pprz_tunnel_SbusFrame, PPRZ_TUNNEL_SBUSFRAME_ID, PPRZ_TUNNEL_SBUSFRAME_SIGNATURE, PPRZ_TUNNEL_SBUSFRAME_MAX_SIZE);
#endif
#endif
