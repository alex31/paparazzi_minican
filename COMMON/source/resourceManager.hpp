/**
 * @file resourceManager.hpp
 * @brief Simple bitmap-based hardware resource manager.
 */
#include <cstdint>
#include <concepts>
#include <type_traits>
#include <bit>
#include <utility>


/** @brief Enumeration of all available hardware resources. */
enum class HWResource : uint8_t {
  PA02, PA03, PA04, PA05, PA06, PA07, PA08, PA09, PA10, PA11, PA12, PA15,
  PB03, PB04, PB07,
  SPI_1, TIM_1, TIM_3, TIM_7, I2C_1, I2C_2, USART_1, USART_2, LPUART_1, 
  F0, F1, F2, F3, F4,
  END
};

/** @brief Error code used when resource allocation fails. */
constexpr msg_t MSG_RESOURCE_CONFLICT = -100;

template <typename T>
concept UnsignedIntegral = std::is_integral_v<T> && std::is_unsigned_v<T>;

template<UnsignedIntegral UNSIGNED = uint32_t>
/** @brief Tracks allocation of HWResource values using a bitmap. */
class HWResourceManager {
private:
  UNSIGNED m_bitmap = 0;
  inline static bool instance_created = false;
  static_assert(std::to_underlying(HWResource::END) <= (sizeof(UNSIGNED) *  8U));
  /** @brief Compute the bit corresponding to a resource. */
  static constexpr UNSIGNED shiftbit(HWResource res) {
    return static_cast<UNSIGNED>(1U) << std::to_underlying(res);
  }

  /** @brief Build a combined mask for a set of resources. */
  static constexpr UNSIGNED mask(std::same_as<HWResource> auto... args)
    requires (sizeof...(args) > 0)
  {
    return (... | shiftbit(args));
  }
  
  
public:
  /** @brief Construct a unique resource manager instance. */
  HWResourceManager() {
    chDbgAssert(!instance_created, "Only one instance of HWResourceManager is allowed");
    instance_created = true;
  }

  /** @brief Reset singleton tracking on destruction. */
  ~HWResourceManager() {
    instance_created = false;
  }

  template<std::same_as<HWResource>... Args>
  /** @brief Attempt to acquire the given resources. */
  bool tryAcquire(Args... resources) {
    UNSIGNED req_mask = mask(resources...);
    if ((m_bitmap & req_mask) != 0) return false;
    m_bitmap |= req_mask;
    return true;
  }

  template<std::same_as<HWResource>... Args>
  /** @brief Release the given resources. */
  void release(Args... resources) {
    UNSIGNED rel_mask = mask(resources...);
    m_bitmap &= ~rel_mask;
  }

  /** @brief Check whether a resource is currently allocated. */
  bool isAllocated(HWResource res) const {
    return (m_bitmap & shiftbit(res)) != 0;
  }

  /** @brief Clear all allocations. */
  void resetAll() {
    m_bitmap = 0;
  }

  /** @brief Return the number of allocated resources. */
  int countAllocated() const {
    return std::popcount(m_bitmap);
  }
};

/// Global resource manager singleton.
inline HWResourceManager boardResource;
