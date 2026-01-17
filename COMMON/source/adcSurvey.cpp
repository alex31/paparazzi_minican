/**
 * @file adcSurvey.cpp
 * @brief ADC sampling, conversion, and threshold monitoring implementation.
 */
#include <ch.h>
#include <hal.h>
#include <utility>
#include <algorithm>
#include "stdutil.h"
#include "adcSurvey.hpp"
#include "adcSamples.hpp"
#include "jobQueue.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "hardwareConf.hpp"

/*
   ° continuous conversion @ 1Khz
   ° ADC Watchdog on psBat and core temp
 */

namespace {
  /** @brief ADC divider resistors that scale battery voltage down. */
   constexpr float resistor_r1 = 2200;
   constexpr float resistor_r2 = 18000;

/** @brief ADC calibration value from Ref Manuel */
  constexpr float ts_cal1_temp = 30;
  constexpr float ts_cal2_temp = 130;
  constexpr float calib_vref = 3.0;
 const uint16_t * const ts_cal1_ptr = (uint16_t *)  0x1FFF75A8;
  const uint16_t * const ts_cal2_ptr = (uint16_t *)  0x1FFF75CA;
  const uint16_t * const vref_int_cal_ptr = (uint16_t *)  0x1FFF75AA;
  const uint16_t&  ts_cal1 = *ts_cal1_ptr;
  const uint16_t&  ts_cal2 = *ts_cal2_ptr;
  const uint16_t&  vref_int_cal = *vref_int_cal_ptr;

  /** jobqueue subsystem that help to pass data from ISR to Thread */
  auto& jq = JobQueue<1, adcerror_t>::instance<512>();

  /** @brief ADC channel indices within the sample buffer. */
  enum class AdcChannel{
    psBat,
    coreTemp, vref, nbChannels};

  AdcSamples<adcsample_t, std::to_underlying(AdcChannel::nbChannels), 8>
			  IN_DMA_SECTION(adcSamples);

  /** @brief Convert a temperature to ADC sample value. */
  uint16_t calculate_tsval(float temp) ;
  /** @brief Convert a voltage to ADC sample value. */
  adcsample_t volts2adc(float v);
  /** @brief Convert ADC sample value to voltage. */
  float adc2volts(adcsample_t sample);
  /** @brief Perform a one-shot ADC conversion. */
  void convert();
  /** @brief Start continuous ADC conversion with thresholds. */
  void startConversion();
  /** @brief Handle ADC watchdog errors in thread context. */
  void adcErrorCb(adcerror_t err);
  Adc::Callback_t *errorCb;

  constexpr ADCConversionGroup adcgrpcfgNoThreshold = {
    .circular     = false,
    .num_channels = adcSamples.nchan(),
    .end_cb       = nullptr,
    .error_cb     = nullptr,
    .cfgr         = ADC_CFGR_CONT,
    .cfgr2        = 0U,
    .tr1          = ADC_TR_DISABLED,
    .tr2          = ADC_TR_DISABLED,
    .tr3          = ADC_TR_DISABLED,
    .awd2cr       = 0, 
    .awd3cr       = 0U,
    .smpr         = {
      ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_640P5),
      ADC_SMPR2_SMP_AN16(ADC_SMPR_SMP_640P5) | ADC_SMPR2_SMP_AN18(ADC_SMPR_SMP_640P5)
    },
    .sqr          = {
      ADC_SQR1_SQ1_N(ADC_CHANNEL_IN1) |
      ADC_SQR1_SQ2_N(ADC_CHANNEL_IN16) | ADC_SQR1_SQ3_N(ADC_CHANNEL_IN18),
      0U,
      0U,
      0U
    }
  };
  ADCConversionGroup adcgrpcfgWithThreshold = adcgrpcfgNoThreshold;
 }                     


namespace Adc {
  /** @brief Start ADC sampling and optional error callback reporting. */
  void start(Callback_t *cb)
  {
    errorCb = cb;
    adcStart(&ADCD1, nullptr);
    adcSTM32EnableVREF(&ADCD1);
    adcSTM32EnableTS(&ADCD1);
    chThdSleepMilliseconds(1); // wait for stability
    convert();

 
    // no ps voltage detected, must run connected to a probe for debug
    const float psBat =  getPsBat(); 
    const bool runOnProbe = psBat < 5.0f;
    if (runOnProbe) {
      DebugTrace("no PS voltage detected [%.2f] , "
		 "assuming run attached to swd probe", psBat);
    }
			   
    const adcsample_t psBatMinSample = volts2adc(psBatMin);
    const adcsample_t psBatMaxSample = volts2adc(psBatMax);
    const adcsample_t coreTempMinSample = calculate_tsval(coreTempMin);
    const adcsample_t coreTempMaxSample = calculate_tsval(coreTempMax);

    adcgrpcfgWithThreshold.circular = true;
    adcgrpcfgWithThreshold.cfgr |=
      runOnProbe ? 0 : (ADC_CFGR_AWD1EN | ADC_CFGR_AWD1SGL |
			 (ADC_CHANNEL_IN1 << ADC_CFGR_AWD1CH_Pos));
    adcgrpcfgWithThreshold.error_cb = [](ADCDriver *, adcerror_t err) {
      chSysLockFromISR();
      jq.submitI([](adcerror_t &_err) {
     	adcErrorCb(_err);
      }, err);
      chSysUnlockFromISR();
    };
    adcgrpcfgWithThreshold.tr1 = ADC_TR(psBatMinSample, psBatMaxSample);
    // threshold 2 (and 3) fields are 8 bits wide only
    adcgrpcfgWithThreshold.tr2 = ADC_TR(coreTempMinSample >> 4,
					coreTempMaxSample >> 4);
    // no threshold on ps if run on probe
    adcgrpcfgWithThreshold.awd2cr = 1 << ADC_CHANNEL_IN16;
    // start continuous conversion
    startConversion();
  }
  
  /** @brief Register an error callback for ADC watchdog events. */
  void setErrorCB(Callback_t *cb)
  {
    errorCb = cb;
  }
  
  /** @brief Return raw battery voltage before calibration. */
  float getPsBatRaw()
  {
    return adc2volts(adcSamples[std::to_underlying(AdcChannel::psBat)]);
  }

  /** @brief Return calibrated battery voltage. */
  float getPsBat()
  {
    const float raw = getPsBatRaw();
    const float scale = param_cget<"adc.psbat.scale">();
    const float bias = param_cget<"adc.psbat.bias">();

    // before MFS is launched and parameters can be used, return raw value
    if (scale != 0)
      return (raw * scale) + bias;
    else
      return raw;
  }

  /** @brief Compute the VCC supply voltage. */
  float getVcc()
  {
    //V REF+ = VREF+_Charac × VREFINT_CAL ⁄ VREFINT_DATA
    return  calib_vref * vref_int_cal /
      adcSamples[std::to_underlying(AdcChannel::vref)];
  }

  /** @brief Convert the temperature sensor sample to degrees Celsius. */
  float getCoreTemp()
  {
    // TS_CALx calibration constants are measured at VREF+ = calib_vref.
    // The ADC code for the temperature sensor scales as 1/VDDA, so we must
    // normalize the measured code back to the calibration voltage:
    //   TS_DATA@cal = TS_DATA@meas * VDDA / calib_vref.
    const float ts_data = adcSamples[std::to_underlying(AdcChannel::coreTemp)]
      * getVcc() / calib_vref;
    const float fact1 = (ts_cal2_temp - ts_cal1_temp) / (ts_cal2 - ts_cal1);
    const float fact2 = (ts_data - ts_cal1);
    return (fact1 * fact2) + ts_cal1_temp;
  }

}


namespace {
  /** @brief Compute the temperature sensor ADC value for a target temperature. */
  uint16_t calculate_tsval(float temp) 
  {
    const float rawv =  ((temp - ts_cal1_temp) / (ts_cal2_temp - ts_cal1_temp))
      * (ts_cal2 - ts_cal1) + ts_cal1;
    return rawv * calib_vref / Adc::getVcc();
  }

  /** @brief Convert a voltage to a raw ADC sample using the divider ratio. */
  adcsample_t volts2adc(float v)
  {
    const adcsample_t sampleMax = (1U << 12) - 1U;
    const float ratio = resistor_r1 / (resistor_r1 + resistor_r2);
    const float adcv = (v/Adc::getVcc()) * sampleMax;
    return adcv * ratio;
  }

  /** @brief Convert a raw ADC sample to a voltage using the divider ratio. */
  float adc2volts(adcsample_t sample)
  {
    const adcsample_t sampleMax = (1U << 12) - 1U;
    const float ratio =(resistor_r1 + resistor_r2) /  resistor_r1 ;
    const float adcv = (static_cast<float>(sample) / sampleMax) * Adc::getVcc();
    return adcv * ratio;
  }

  /** @brief Perform a blocking ADC conversion. */
  void convert()
  {
    adcConvert(&ADCD1, &adcgrpcfgNoThreshold,
	       adcSamples.data(), adcSamples.depth());
  }

  /** @brief Start continuous ADC conversion with thresholds enabled. */
  void startConversion()
  {
    adcStartConversion(&ADCD1, &adcgrpcfgWithThreshold,
		       adcSamples.data(), adcSamples.depth());
  }


  // this callback is called in thread context via jobqueue module
  /** @brief Handle ADC watchdog events in a job queue context. */
  void adcErrorCb(adcerror_t err)
  {
    if (err & (ADC_ERR_AWD1 | ADC_ERR_AWD2)) {
      if (errorCb != nullptr) {
	const float psBat = Adc::getPsBat();
	const float coreTemp = Adc::getCoreTemp();
	// if one of average value is out of range, then invoque
	// error callback
	if ((std::clamp(psBat, psBatMin, psBatMax) != psBat) or
	    (std::clamp(coreTemp, coreTempMin, coreTempMax) != coreTemp)) {
	  errorCb(Adc::getPsBat(), Adc::getCoreTemp());
	}
	//convert();
      }
    }
    chThdSleepMilliseconds(10);
    startConversion();
  }
}
