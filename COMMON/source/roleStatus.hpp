#pragma once
#include "frozen/map.h"
#include "frozen/string.h"
#include "etl/string.h"
#include <etl/to_string.h>
#include <array>

// C++23: constexpr std::to_array permet de déduire la taille à partir d'une liste
template <typename K, typename V, std::size_t N>
constexpr frozen::map<K, V, N> make_frozen_map(std::array<std::pair<K, V>, N> arr) {
    return frozen::map<K, V, N>(arr);
}


#define MKP(e)  std::pair{e, #e}

struct DeviceStatus {
  enum Source : uint8_t {
    ALL, MFS, RESOURCE, SERVO_ROLE, SERVO_PWM, SERVO_SMART,
    MPL3115A2, ESC_DSHOT, RC_SBUS, TELEMETRY_TUNNEL, GPS_ROLE,
    MAG_QMC5883, FIRMWARE_UPDATE, I2C, SPI, MEMORY, NUM_SOURCES
  };
  enum Error : uint8_t {
    OK, NOT_FOUND, INVALID_PARAM, HETEROGENEOUS_BAUDS,
    NOT_RESPONDING, CONFLICT, I2C_TIMOUT, I2C_FREQ_INVALID, NB_ROLE_TOO_LARGE,
    INVALID_PWM_MASK, HEAP_FULL, DMA_HEAP_FULL, NUM_ERRORS
  };
  constexpr DeviceStatus(Source s, Error e = OK,
	       uint16_t spe = 0) : source(s), err(e), specific(spe) {}
  constexpr DeviceStatus(const DeviceStatus&) = default;
  void set(Error e, uint16_t spe=0) {
    err = e; specific = spe;
  }
  
  DeviceStatus() = delete;
  // convert to true if no error
  explicit operator bool() const {return err == OK;}
  
  Source source;
  Error  err;
  uint16_t specific;
  
  static constexpr auto srcName = make_frozen_map(std::to_array({
	MKP(ALL),
	MKP(MFS),
	MKP(RESOURCE),
	MKP(SERVO_ROLE),
	MKP(SERVO_PWM),
	MKP(SERVO_SMART),
	MKP(MPL3115A2),
	MKP(ESC_DSHOT),
	MKP(RC_SBUS),
	MKP(TELEMETRY_TUNNEL),
	MKP(GPS_ROLE),
	MKP(MAG_QMC5883),
	MKP(FIRMWARE_UPDATE),
	MKP(I2C),
	MKP(SPI),
	MKP(MEMORY)
      }));

  static constexpr auto errName = make_frozen_map(std::to_array({
	MKP(OK),
	MKP(NOT_FOUND),
	MKP(INVALID_PARAM),
	MKP(HETEROGENEOUS_BAUDS),
	MKP(NOT_RESPONDING),
	MKP(I2C_TIMOUT),
	MKP(I2C_FREQ_INVALID),
	MKP(NB_ROLE_TOO_LARGE),
	MKP(INVALID_PWM_MASK),
	MKP(HEAP_FULL),
	MKP(DMA_HEAP_FULL),
	MKP(CONFLICT)
      }));

  static_assert(srcName.size() == NUM_SOURCES);
  static_assert(errName.size() == NUM_ERRORS);
  
  etl::string<64> describe() const {
    etl::string<64> out;
    out = "SRC=";
    out += srcName.at(source);
    out += " ERR=";
    out += errName.at(err);
    out += " SPEC=";
    
    etl::to_string(specific, out, true);  // en base 10, append instead of overwrite
    return out;
  }
};

static_assert(sizeof(DeviceStatus) == sizeof(int));
