#include <ch.h>
#include <hal.h>
#include <utility>
#include <algorithm>
#include "stdutil.h"
#include "adcSurvey.hpp"
#include "adcSamples.hpp"
#include "jobQueue.hpp"

/*
   ° continuous conversion @ 1Khz
   ° ADC Watchdog on psBat and core temp

 */

/*
  TODO : 




  */
namespace {
  constexpr float psBatMin = 6;
  constexpr float psBatMax = 26;
  constexpr float coreTempMin = -10.0f;
  constexpr float coreTempMax = 60.0f;

  constexpr float ts_cal1_temp = 30;
  constexpr float ts_cal2_temp = 130;
  constexpr float calib_vref = 3.0;
  constexpr float vcc = 3.3f;
  constexpr float resistor_r1 = 2200;
  constexpr float resistor_r2 = 18000;

  const uint16_t * const ts_cal1_ptr = (uint16_t *)  0x1FFF75A8;
  const uint16_t * const ts_cal2_ptr = (uint16_t *)  0x1FFF75CA;
  const uint16_t * const vref_int_cal_ptr = (uint16_t *)  0x1FFF75AA;
  const uint16_t&  ts_cal1 = *ts_cal1_ptr;
  const uint16_t&  ts_cal2 = *ts_cal2_ptr;
  const uint16_t&  vref_int_cal = *vref_int_cal_ptr;

  auto& jq = JobQueue<1, adcerror_t>::instance<512>();

  enum class AdcChannel{
    psBat,
#ifdef	BOARD_ENAC_MICROCANv1
    address, 
#endif
    coreTemp, vref, nbChannels};

  AdcSamples<adcsample_t, std::to_underlying(AdcChannel::nbChannels), 8>
			  IN_DMA_SECTION(adcSamples);

  uint16_t calculate_tsval(float temp) ;
  adcsample_t volts2adc(float v);
  float adc2volts(adcsample_t sample);
  void convert();
  void startConversion();
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
#ifdef	BOARD_ENAC_MICROCANv1  
      ADC_SMPR1_SMP_AN2(ADC_SMPR_SMP_640P5) |
#endif
      ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_640P5),
      ADC_SMPR2_SMP_AN16(ADC_SMPR_SMP_640P5) | ADC_SMPR2_SMP_AN18(ADC_SMPR_SMP_640P5)
    },
    .sqr          = {
#ifdef	BOARD_ENAC_MICROCANv1
      ADC_SQR1_SQ1_N(ADC_CHANNEL_IN1) |
      ADC_SQR1_SQ2_N(ADC_CHANNEL_IN2) |
      ADC_SQR1_SQ3_N(ADC_CHANNEL_IN16) | ADC_SQR1_SQ4_N(ADC_CHANNEL_IN18),
#else
      ADC_SQR1_SQ1_N(ADC_CHANNEL_IN1) |
      ADC_SQR1_SQ2_N(ADC_CHANNEL_IN16) | ADC_SQR1_SQ3_N(ADC_CHANNEL_IN18),
#endif
      0U,
      0U,
      0U
    }
  };
  ADCConversionGroup adcgrpcfgWithThreshold = adcgrpcfgNoThreshold;
 }                     


namespace Adc {
  void start(Callback_t *cb)
  {
    errorCb = cb;
    adcStart(&ADCD1, nullptr);
    adcSTM32EnableVREF(&ADCD1);
    adcSTM32EnableTS(&ADCD1);
    convert();

    // no ps voltage detected, must run connected to a probe for debug
    const float psBat =  getPsBat() ;
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
  
  
#ifdef	BOARD_ENAC_MICROCANv1
  uint8_t getAddress()
  {
    return adcSamples[std::to_underlying(AdcChannel::address)] >> 8;
  }
#endif
  
  float getPsBat()
  {
    return adc2volts(adcSamples[std::to_underlying(AdcChannel::psBat)]);
  }

  float getVcc()
  {
    //V REF+ = VREF+_Charac × VREFINT_CAL ⁄ VREFINT_DATA
    return  calib_vref * vref_int_cal /
      adcSamples[std::to_underlying(AdcChannel::vref)];
  }

  float getCoreTemp()
  {
    const float ts_data = adcSamples[std::to_underlying(AdcChannel::coreTemp)]
      * getVcc() / calib_vref;
    const float fact1 = (ts_cal2_temp - ts_cal1_temp) / (ts_cal2 - ts_cal1);
    const float fact2 = (ts_data - ts_cal1);
    return (fact1 * fact2) + ts_cal1_temp;
  }

}


namespace {
  uint16_t calculate_tsval(float temp) 
  {
    const float rawv =  ((temp - ts_cal1_temp) / (ts_cal2_temp - ts_cal1_temp))
      * (ts_cal2 - ts_cal1) + ts_cal1;
    return rawv * calib_vref / Adc::getVcc();
  }

  adcsample_t volts2adc(float v)
  {
    const adcsample_t sampleMax = (1U << 12) - 1U;
    const float ratio = resistor_r1 / (resistor_r1 + resistor_r2);
    const float adcv = (v/Adc::getVcc()) * sampleMax;
    return adcv * ratio;
  }

  float adc2volts(adcsample_t sample)
  {
    const adcsample_t sampleMax = (1U << 12) - 1U;
    const float ratio =(resistor_r1 + resistor_r2) /  resistor_r1 ;
    const float adcv = (static_cast<float>(sample) / sampleMax) * Adc::getVcc();
    return adcv * ratio;
  }

  void convert()
  {
    adcConvert(&ADCD1, &adcgrpcfgNoThreshold,
	       adcSamples.data(), adcSamples.depth());
  }

  void startConversion()
  {
    adcStartConversion(&ADCD1, &adcgrpcfgWithThreshold,
		       adcSamples.data(), adcSamples.depth());
  }


  // this callback is called in thread context via jobqueue module
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
