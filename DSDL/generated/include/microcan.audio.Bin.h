#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define MICROCAN_AUDIO_BIN_MAX_SIZE 4
#define MICROCAN_AUDIO_BIN_SIGNATURE (0xD033AE6CAFE865E5ULL)


struct microcan_audio_Bin {
    uint16_t frequency_hz;
    uint16_t level;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t _microcan_audio_Bin_encode(struct microcan_audio_Bin* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool _microcan_audio_Bin_decode(const CanardRxTransfer* transfer, struct microcan_audio_Bin* msg);

static inline uint32_t microcan_audio_Bin_encode(struct microcan_audio_Bin* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {

    return _microcan_audio_Bin_encode(msg, buffer
#if CANARD_ENABLE_TAO_OPTION
    , tao
#endif
    );

}

static inline bool microcan_audio_Bin_decode(const CanardRxTransfer* transfer, struct microcan_audio_Bin* msg) {

    return _microcan_audio_Bin_decode(transfer, msg);

}

#if defined(CANARD_DSDLC_INTERNAL)
static inline void __microcan_audio_Bin_encode(uint8_t* buffer, uint32_t* bit_ofs, struct microcan_audio_Bin* msg, bool tao);
static inline bool __microcan_audio_Bin_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct microcan_audio_Bin* msg, bool tao);
void __microcan_audio_Bin_encode(uint8_t* buffer, uint32_t* bit_ofs, struct microcan_audio_Bin* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->frequency_hz);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->level);
    *bit_ofs += 16;
}

/*
 decode microcan_audio_Bin, return true on failure, false on success
*/
bool __microcan_audio_Bin_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct microcan_audio_Bin* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->frequency_hz);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->level);
    *bit_ofs += 16;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct microcan_audio_Bin sample_microcan_audio_Bin_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
