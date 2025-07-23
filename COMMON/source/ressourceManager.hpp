#include <cstdint>
#include <concepts>
#include <type_traits>
#include <bit>
#include <utility>


// Enum of all avalaible resources
// SPI_1 instead of SPI1 to avoid CMSIS conflict
enum class HWResource : uint8_t {
  PA02, PA03, PA04, PA05, PA06, PA07, PA08, PA09, PA10, PA11, PA12, PA15,
  PB03, PB04, PB07,
  SPI_1, TIM_1, TIM_7, I2C_1, USART_2, LPUART_1, 
  F0, F1, F2, F3, F4,
  END
};

constexpr msg_t MSG_RESOURCE_CONFLICT = -100;

template <typename T>
concept UnsignedIntegral = std::is_integral_v<T> && std::is_unsigned_v<T>;

template<UnsignedIntegral UNSIGNED = uint32_t>
class HWResourceManager {
private:
  UNSIGNED m_bitmap = 0;
  inline static bool instance_created = false;
  static_assert(std::to_underlying(HWResource::END) <= (sizeof(UNSIGNED) *  8U));
  static constexpr UNSIGNED shiftbit(HWResource res) {
    return static_cast<UNSIGNED>(1U) << std::to_underlying(res);
  }

  template<std::same_as<HWResource>... Args>
  static constexpr UNSIGNED mask(HWResource first, Args... rest) {
    return shiftbit(first) | mask(rest...);
  }

  static constexpr UNSIGNED mask(HWResource res) {
    return shiftbit(res);
  }

public:
  HWResourceManager() {
    chDbgAssert(!instance_created, "Only one instance of HWResourceManager is allowed");
    instance_created = true;
  }

  ~HWResourceManager() {
    instance_created = false;
  }

  template<std::same_as<HWResource>... Args>
  bool try_acquire(Args... resources) {
    UNSIGNED req_mask = mask(resources...);
    if ((m_bitmap & req_mask) != 0) return false;
    m_bitmap |= req_mask;
    return true;
  }

  template<std::same_as<HWResource>... Args>
  void release(Args... resources) {
    UNSIGNED rel_mask = mask(resources...);
    m_bitmap &= ~rel_mask;
  }

  bool is_allocated(HWResource res) const {
    return (m_bitmap & shiftbit(res)) != 0;
  }

  void reset_all() {
    m_bitmap = 0;
  }

  int count_allocated() const {
    return std::popcount(m_bitmap);
  }
};

// Singleton global
inline HWResourceManager boardResource;

