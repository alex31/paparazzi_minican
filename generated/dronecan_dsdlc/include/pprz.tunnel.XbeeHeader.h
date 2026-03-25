#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <pprz.tunnel.Empty.h>
#include <pprz.tunnel.XbeeHeaderRx.h>
#include <pprz.tunnel.XbeeHeaderTx.h>


#define PPRZ_TUNNEL_XBEEHEADER_MAX_SIZE 6
#define PPRZ_TUNNEL_XBEEHEADER_SIGNATURE (0xD92C646982F6634ULL)

enum pprz_tunnel_XbeeHeader_type_t {
    PPRZ_TUNNEL_XBEEHEADER_TX,
    PPRZ_TUNNEL_XBEEHEADER_RX,
    PPRZ_TUNNEL_XBEEHEADER_NONE,
};


struct pprz_tunnel_XbeeHeader {
    enum pprz_tunnel_XbeeHeader_type_t union_tag;
    union {
        struct pprz_tunnel_XbeeHeaderTx tx;
        struct pprz_tunnel_XbeeHeaderRx rx;
        struct pprz_tunnel_Empty none;
    };
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t pprz_tunnel_XbeeHeader_encode(struct pprz_tunnel_XbeeHeader* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool pprz_tunnel_XbeeHeader_decode(const CanardRxTransfer* transfer, struct pprz_tunnel_XbeeHeader* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _pprz_tunnel_XbeeHeader_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_XbeeHeader* msg, bool tao);
static inline bool _pprz_tunnel_XbeeHeader_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_XbeeHeader* msg, bool tao);
void _pprz_tunnel_XbeeHeader_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_XbeeHeader* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    uint8_t union_tag = msg->union_tag;
    canardEncodeScalar(buffer, *bit_ofs, 2, &union_tag);
    *bit_ofs += 2;

    switch(msg->union_tag) {
        case PPRZ_TUNNEL_XBEEHEADER_TX: {
            _pprz_tunnel_XbeeHeaderTx_encode(buffer, bit_ofs, &msg->tx, tao);
            break;
        }
        case PPRZ_TUNNEL_XBEEHEADER_RX: {
            _pprz_tunnel_XbeeHeaderRx_encode(buffer, bit_ofs, &msg->rx, tao);
            break;
        }
        case PPRZ_TUNNEL_XBEEHEADER_NONE: {
            _pprz_tunnel_Empty_encode(buffer, bit_ofs, &msg->none, tao);
            break;
        }
    }
}

/*
 decode pprz_tunnel_XbeeHeader, return true on failure, false on success
*/
bool _pprz_tunnel_XbeeHeader_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_XbeeHeader* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    uint8_t union_tag;
    canardDecodeScalar(transfer, *bit_ofs, 2, false, &union_tag);
    *bit_ofs += 2;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (union_tag >= 3) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    msg->union_tag = (enum pprz_tunnel_XbeeHeader_type_t)union_tag;

    switch(msg->union_tag) {
        case PPRZ_TUNNEL_XBEEHEADER_TX: {
            if (_pprz_tunnel_XbeeHeaderTx_decode(transfer, bit_ofs, &msg->tx, tao)) {return true;}
            break;
        }

        case PPRZ_TUNNEL_XBEEHEADER_RX: {
            if (_pprz_tunnel_XbeeHeaderRx_decode(transfer, bit_ofs, &msg->rx, tao)) {return true;}
            break;
        }

        case PPRZ_TUNNEL_XBEEHEADER_NONE: {
            if (_pprz_tunnel_Empty_decode(transfer, bit_ofs, &msg->none, tao)) {return true;}
            break;
        }

    }
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct pprz_tunnel_XbeeHeader sample_pprz_tunnel_XbeeHeader_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
