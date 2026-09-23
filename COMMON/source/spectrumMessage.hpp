#pragma once

#include <microcan.audio.Spectrum.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace SpectrumWire {
  constexpr size_t headerBytes = 2U;
  constexpr size_t countBytes = 1U;
  constexpr size_t binBytes = 2U * sizeof(uint16_t);
  constexpr size_t maxBinsForPayload(size_t payloadBytes)
  {
    return payloadBytes < headerBytes + countBytes ? 0U :
      (payloadBytes - headerBytes - countBytes) / binBytes;
  }
  constexpr size_t maxBins = maxBinsForPayload(CANARD_MAX_TRANSFER_PAYLOAD_LEN);
  static_assert(maxBins == 255U, "Update the DSDL bound when the transport limit changes");
  static_assert(MICROCAN_AUDIO_SPECTRUM_MAX_SIZE ==
                headerBytes + countBytes + binBytes * maxBins);
}

/** Send-side view of the canonical Spectrum schema with bounded local storage.
 * A producer of ten bins reserves ten slots, not the 255-slot protocol maximum.
 * The canonical generated encoder clears its entire 1023-byte output buffer,
 * so this small encoder is checked byte-for-byte against it in host tests.
 * Use one specialization per node/subject (Node owns a transfer ID per type).
 */
template<size_t Capacity>
struct SpectrumMessage {
  static_assert(Capacity > 0U && Capacity <= SpectrumWire::maxBins);
  uint8_t adc_bits = 0U;
  uint8_t status = 0U;
  uint8_t count = 0U;
  std::array<microcan_audio_Bin, Capacity> bins = {};

  struct cxx_iface {
    static constexpr uint16_t ID = MICROCAN_AUDIO_SPECTRUM_ID;
    static constexpr uint64_t SIGNATURE = MICROCAN_AUDIO_SPECTRUM_SIGNATURE;
    static constexpr uint16_t MAX_SIZE = SpectrumWire::headerBytes +
      SpectrumWire::countBytes + SpectrumWire::binBytes * Capacity;

    static uint32_t encode(SpectrumMessage *message, uint8_t *buffer, bool tao)
    {
      const uint8_t count = static_cast<uint8_t>(std::min<size_t>(message->count, Capacity));
      std::memset(buffer, 0, MAX_SIZE);
      canardEncodeScalar(buffer, 0U, 8U, &message->adc_bits);
      canardEncodeScalar(buffer, 8U, 8U, &message->status);
      uint32_t offset = SpectrumWire::headerBytes * 8U;
      if (not tao) {
        canardEncodeScalar(buffer, offset, 8U, &count);
        offset += 8U;
      }
      for (size_t i = 0U; i < count; ++i) {
        canardEncodeScalar(buffer, offset, 16U, &message->bins[i].frequency_hz);
        canardEncodeScalar(buffer, offset + 16U, 16U, &message->bins[i].level);
        offset += SpectrumWire::binBytes * 8U;
      }
      return offset / 8U;
    }
  };
};
