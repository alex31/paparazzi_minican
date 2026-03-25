#define CANARD_DSDLC_INTERNAL
#include <pprz.tunnel.SbusFrame.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t pprz_tunnel_SbusFrame_encode(struct pprz_tunnel_SbusFrame* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, PPRZ_TUNNEL_SBUSFRAME_MAX_SIZE);
    _pprz_tunnel_SbusFrame_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

/*
  return true if the decode is invalid
 */
bool pprz_tunnel_SbusFrame_decode(const CanardRxTransfer* transfer, struct pprz_tunnel_SbusFrame* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > PPRZ_TUNNEL_SBUSFRAME_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_pprz_tunnel_SbusFrame_decode(transfer, &bit_ofs, msg,
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    )) {
        return true; /* invalid payload */
    }

    const uint32_t byte_len = (bit_ofs+7U)/8U;
#if CANARD_ENABLE_TAO_OPTION
    // if this could be CANFD then the dlc could indicating more bytes than
    // we actually have
    if (!transfer->tao) {
        return byte_len > transfer->payload_len;
    }
#endif
    return byte_len != transfer->payload_len;
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct pprz_tunnel_SbusFrame sample_pprz_tunnel_SbusFrame_msg(void) {
    struct pprz_tunnel_SbusFrame msg;

    msg.flags = (uint8_t)random_bitlen_unsigned_val(8);
    msg.channels.len = (uint8_t)random_range_unsigned_val(0, 16);
    for (size_t i=0; i < msg.channels.len; i++) {
        msg.channels.data[i] = (uint16_t)random_bitlen_unsigned_val(11);
    }
    return msg;
}
#endif
