#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define PPRZ_TUNNEL_EMPTY_MAX_SIZE 0
#define PPRZ_TUNNEL_EMPTY_SIGNATURE (0x1D692E22088968A2ULL)


struct pprz_tunnel_Empty {
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t pprz_tunnel_Empty_encode(struct pprz_tunnel_Empty* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool pprz_tunnel_Empty_decode(const CanardRxTransfer* transfer, struct pprz_tunnel_Empty* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _pprz_tunnel_Empty_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_Empty* msg, bool tao);
static inline bool _pprz_tunnel_Empty_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_Empty* msg, bool tao);
void _pprz_tunnel_Empty_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_tunnel_Empty* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode pprz_tunnel_Empty, return true on failure, false on success
*/
bool _pprz_tunnel_Empty_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_tunnel_Empty* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct pprz_tunnel_Empty sample_pprz_tunnel_Empty_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
