#pragma once

#include <array>
#include "stdutil.h"


template <typename SMPL_T, size_t NCHAN, size_t DEPTH>
class AdcSamples {
public:
  SMPL_T *data()  {return samples.data();}
  const SMPL_T *cdata() const  {return samples.data();}
  // when channel and depth are given : return sample
  SMPL_T operator[](size_t chan, size_t depth) const {
    return samples[(depth * NCHAN) +  chan];
  }
  // when only channel is given :
  // return average for all depths samples for that channel
  SMPL_T operator[](size_t chan) const {
    uint32_t sum = 0;
    for (size_t depth=0; depth < DEPTH; depth++) {
      sum += (*this)[chan, depth];
    }
    return sum / DEPTH;
  }
  
  constexpr size_t depth() const {return DEPTH;}
  constexpr size_t nchan() const {return NCHAN;}
  
private:
  std::array<SMPL_T, NCHAN * DEPTH> samples = {};
};
