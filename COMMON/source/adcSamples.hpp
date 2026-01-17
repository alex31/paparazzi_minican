/**
 * @file adcSamples.hpp
 * @brief ADC sample buffer helper template.
 */
#pragma once

#include <array>
#include "stdutil.h"


/**
 * @brief Fixed-size ADC sample buffer with per-channel averaging.
 *
 * @tparam SMPL_T Sample type.
 * @tparam NCHAN  Number of channels.
 * @tparam DEPTH  Samples per channel.
 */
template <typename SMPL_T, size_t NCHAN, size_t DEPTH>
class AdcSamples {
public:
  /** @brief Return a mutable pointer to the raw sample array. */
  SMPL_T *data()  {return samples.data();}
  /** @brief Return a const pointer to the raw sample array. */
  const SMPL_T *cdata() const  {return samples.data();}
  // when channel and depth are given : return sample
  /** @brief Return the sample for a channel and depth index. */
  SMPL_T operator[](size_t chan, size_t depth) const {
    return samples[(depth * NCHAN) +  chan];
  }
  // when only channel is given :
  // return average for all depths samples for that channel
  /** @brief Return the averaged sample for a channel. */
  SMPL_T operator[](size_t chan) const {
    uint32_t sum = 0;
    for (size_t depth=0; depth < DEPTH; depth++) {
      sum += (*this)[chan, depth];
    }
    return sum / DEPTH;
  }
  
  /** @brief Number of samples per channel. */
  constexpr size_t depth() const {return DEPTH;}
  /** @brief Number of channels. */
  constexpr size_t nchan() const {return NCHAN;}
  
private:
  std::array<SMPL_T, NCHAN * DEPTH> samples = {};
};
