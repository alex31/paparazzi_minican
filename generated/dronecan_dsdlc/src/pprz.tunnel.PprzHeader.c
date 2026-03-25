#define CANARD_DSDLC_INTERNAL
#include <pprz.tunnel.PprzHeader.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t pprz_tunnel_PprzHeader_encode(struct pprz_tunnel_PprzHeader* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, PPRZ_TUNNEL_PPRZHEADER_MAX_SIZE);
    _pprz_tunnel_PprzHeader_encode(buffer, &bit_ofs, msg, 
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
bool pprz_tunnel_PprzHeader_decode(const CanardRxTransfer* transfer, struct pprz_tunnel_PprzHeader* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > PPRZ_TUNNEL_PPRZHEADER_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_pprz_tunnel_PprzHeader_decode(transfer, &bit_ofs, msg,
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
struct pprz_tunnel_PprzHeader sample_pprz_tunnel_PprzHeader_msg(void) {
    struct pprz_tunnel_PprzHeader msg;

    msg.source = (uint8_t)random_bitlen_unsigned_val(8);
    msg.destination = (uint8_t)random_bitlen_unsigned_val(8);
    msg.classId = (uint8_t)random_bitlen_unsigned_val(4);
    msg.componentId = (uint8_t)random_bitlen_unsigned_val(4);
    msg.msgId = (uint8_t)random_bitlen_unsigned_val(8);
    return msg;
}
#endif
