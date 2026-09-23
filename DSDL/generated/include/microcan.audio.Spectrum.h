#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <microcan.audio.Bin.h>


#define MICROCAN_AUDIO_SPECTRUM_MAX_SIZE 1023
#define MICROCAN_AUDIO_SPECTRUM_SIGNATURE (0x3F89C36ADB5BF657ULL)
#define MICROCAN_AUDIO_SPECTRUM_ID 20900

#define MICROCAN_AUDIO_SPECTRUM_STATUS_VALID 1
#define MICROCAN_AUDIO_SPECTRUM_STATUS_CLIPPED 2
#define MICROCAN_AUDIO_SPECTRUM_STATUS_DISCONTINUITY 4

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class microcan_audio_Spectrum_cxx_iface;
#endif

struct microcan_audio_Spectrum {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = microcan_audio_Spectrum_cxx_iface;
#endif
    uint8_t adc_bits;
    uint8_t status;
    struct { uint8_t len; struct microcan_audio_Bin data[255]; }bins;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t _microcan_audio_Spectrum_encode(struct microcan_audio_Spectrum* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool _microcan_audio_Spectrum_decode(const CanardRxTransfer* transfer, struct microcan_audio_Spectrum* msg);

static inline uint32_t microcan_audio_Spectrum_encode(struct microcan_audio_Spectrum* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {

    return _microcan_audio_Spectrum_encode(msg, buffer
#if CANARD_ENABLE_TAO_OPTION
    , tao
#endif
    );

}

static inline bool microcan_audio_Spectrum_decode(const CanardRxTransfer* transfer, struct microcan_audio_Spectrum* msg) {

    return _microcan_audio_Spectrum_decode(transfer, msg);

}

#if defined(CANARD_DSDLC_INTERNAL)
static inline void __microcan_audio_Spectrum_encode(uint8_t* buffer, uint32_t* bit_ofs, struct microcan_audio_Spectrum* msg, bool tao);
static inline bool __microcan_audio_Spectrum_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct microcan_audio_Spectrum* msg, bool tao);
void __microcan_audio_Spectrum_encode(uint8_t* buffer, uint32_t* bit_ofs, struct microcan_audio_Spectrum* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->adc_bits);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->status);
    *bit_ofs += 8;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t bins_len = msg->bins.len > 255 ? 255 : msg->bins.len;
#pragma GCC diagnostic pop
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &bins_len);
        *bit_ofs += 8;
    }
    for (size_t i=0; i < bins_len; i++) {
        __microcan_audio_Bin_encode(buffer, bit_ofs, &msg->bins.data[i], false);
    }
}

/*
 decode microcan_audio_Spectrum, return true on failure, false on success
*/
bool __microcan_audio_Spectrum_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct microcan_audio_Spectrum* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->adc_bits);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->status);
    *bit_ofs += 8;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->bins.len);
        *bit_ofs += 8;
    }


    if (tao) {
        msg->bins.len = 0;
        size_t max_len = 255;
        uint32_t max_bits = (transfer->payload_len*8)-7; // TAO elements must be >= 8 bits
        while (max_bits > *bit_ofs) {
            if (!max_len-- || __microcan_audio_Bin_decode(transfer, bit_ofs, &msg->bins.data[msg->bins.len], false)) {return true;}
            msg->bins.len++;
        }
    } else {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        if (msg->bins.len > 255) {
            return true; /* invalid value */
        }
#pragma GCC diagnostic pop
        for (size_t i=0; i < msg->bins.len; i++) {
            if (__microcan_audio_Bin_decode(transfer, bit_ofs, &msg->bins.data[i], false)) {return true;}
        }
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct microcan_audio_Spectrum sample_microcan_audio_Spectrum_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
BROADCAST_MESSAGE_CXX_IFACE(microcan_audio_Spectrum, MICROCAN_AUDIO_SPECTRUM_ID, MICROCAN_AUDIO_SPECTRUM_SIGNATURE, MICROCAN_AUDIO_SPECTRUM_MAX_SIZE);
#endif
#endif
