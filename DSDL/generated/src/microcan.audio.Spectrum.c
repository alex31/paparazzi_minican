#define CANARD_DSDLC_INTERNAL
#include <microcan.audio.Spectrum.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t _microcan_audio_Spectrum_encode(struct microcan_audio_Spectrum* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, MICROCAN_AUDIO_SPECTRUM_MAX_SIZE);
    __microcan_audio_Spectrum_encode(buffer, &bit_ofs, msg,
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
bool _microcan_audio_Spectrum_decode(const CanardRxTransfer* transfer, struct microcan_audio_Spectrum* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > MICROCAN_AUDIO_SPECTRUM_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (__microcan_audio_Spectrum_decode(transfer, &bit_ofs, msg,
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
struct microcan_audio_Spectrum sample_microcan_audio_Spectrum_msg(void) {
    struct microcan_audio_Spectrum msg;

    msg.adc_bits = (uint8_t)random_bitlen_unsigned_val(8);
    msg.status = (uint8_t)random_bitlen_unsigned_val(8);
    msg.bins.len = (uint8_t)random_range_unsigned_val(0, 255);
    for (size_t i=0; i < msg.bins.len; i++) {
        msg.bins.data[i] = sample_microcan_audio_Bin_msg();
    }
    return msg;
}
#endif
