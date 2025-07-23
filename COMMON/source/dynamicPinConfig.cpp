#include "dynamicPinConfig.hpp"
#include "stdutil.h"
#include "UAVCAN/persistantParam.hpp"

#ifdef  BOARD_ENAC_MINICANv3
#define LINE_F4_a	LINE_SPI_CS	
#define LINE_F3_a	LINE_SPI_SCK	
#define LINE_F2_a	LINE_SPI_MISO   
#define LINE_F1_a	LINE_SPI_MOSI   
#define LINE_F4_b	LINE_SRV1	
#define LINE_F3_b	LINE_SRV2	
#define LINE_F2_b	LINE_SRV3	
#define LINE_F1_b	LINE_SRV4	
#define LINE_F0_a	LINE_SWITCH_AD2 
#define LINE_F1_c	LINE_I2C_SCL	
#define LINE_F0_b	LINE_UART_TX    
#define LINE_F0_c	LINE_I2C_SDA    
#elifdef  BOARD_ENAC_MINICANv4
#define LINE_F4_a	LINE_SPI_PERIPH_CS
#define LINE_F3_a	LINE_SPI_SCK	
#define LINE_F2_a	LINE_SPI_MISO   
#define LINE_F1_a	LINE_SPI_MOSI   
#define LINE_F4_b	LINE_SRV1	
#define LINE_F3_b	LINE_SRV2	
#define LINE_F2_b	LINE_SRV3	
#define LINE_F1_b	LINE_SRV4	
#define LINE_F0_a	LINE_PULLUP_SCL
#define LINE_F1_c	LINE_I2C_SCL	
#define LINE_F0_b	LINE_UART_TX    
#define LINE_F0_c	LINE_I2C_SDA    
#endif

namespace {
  // RAII port configuration save/restore
  class gpio_config_t {
    using gpio_t = GPIO_TypeDef *;
    const gpio_t gpio_port_a = reinterpret_cast<GPIO_TypeDef *>(GPIOA_BASE);
    const gpio_t gpio_port_b = reinterpret_cast<GPIO_TypeDef *>(GPIOB_BASE);
    GPIO_TypeDef a, b;
  
  public:
    gpio_config_t() {save();}
  
    void restore() {
      *gpio_port_a = a;
      *gpio_port_b = b;
    }
  
  private:
    void save() {
      a = *gpio_port_a;
      b = *gpio_port_b;
    }
  };
}

namespace DynPin {
  bool i2cUnhangBus(I2CDriver *i2cd)
  {
    bool sdaReleased;
    gpio_config_t context;
    ioline_t sdaLine, sclLine;

    if (i2cd == &I2CD1) {
      sdaLine = LINE_F0_c;
      sclLine = LINE_F1_c;
    } else {
      sdaLine = LINE_F4_b;
      sclLine = LINE_F3_b;
    }
    
    palSetLineMode (sdaLine, PAL_MODE_INPUT);
    chThdSleepMicroseconds(100);
    sdaReleased = palReadLine (sdaLine) == PAL_HIGH;
    uint32_t currentInput ;
    if (sdaReleased) 
      goto end;
  
    palSetLineMode (sclLine, PAL_MODE_INPUT);
    chThdSleepMicroseconds(100);
    currentInput = palReadLine (sclLine) == PAL_HIGH;
    palSetLineMode (sclLine, PAL_MODE_OUTPUT_PUSHPULL);
    palWriteLine (sclLine, currentInput);
    chThdSleepMicroseconds(100);
  
    for (uint8_t i=0; i<=8; i++) {
      chSysPolledDelayX (US2RTC(STM32_SYSCLK, 10)) ; // 10µs : 100 khz
      palToggleLine (sclLine);
      chSysPolledDelayX (US2RTC(STM32_SYSCLK, 10)) ; // 10µs : 100 khz
      palToggleLine (sclLine);
      chSysPolledDelayX (US2RTC(STM32_SYSCLK, 10)) ; // 10µs : 100 khz
    
      sdaReleased = palReadLine (sdaLine) == PAL_HIGH;
      if (sdaReleased) 
	break;
    }
  
  end:
    context.restore();
    return sdaReleased; 
  }

  void i2cActivatePullup()
  {
    // activate PULLUP on SDA, SCL depending on eeprom configuration
    palSetLineMode(LINE_PULLUP_SCL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode(LINE_PULLUP_SDA, PAL_MODE_OUTPUT_PUSHPULL);

    if (PARAM_CGET("bus.i2c.pullup_resistor")) {
      palSetLine(LINE_PULLUP_SCL);
      palSetLine(LINE_PULLUP_SDA);
    } else {
      palClearLine(LINE_PULLUP_SCL);
      palClearLine(LINE_PULLUP_SDA);
    }
  }

}

#ifdef  BOARD_ENAC_MICROCANv1

namespace {
  void scenario_UART();
  void scenario_I2C();
  void scenario_SPI();
  void scenario_PWM();
  void scenario_DSHOT();
}



namespace DynPin {

  void inactiveAllSharedPins()
  {
    BOARD_GROUP_DECLFOREACH(pin, DYNAMIC_FUNCTION_PIN) {
      palSetLineMode(pin, PAL_MODE_INPUT);
    }
  }

  void setScenario(Scenario s)
  {
    // verify that we set the scenion only once
    static bool scenarioSet = false;
    chDbgAssert(scenarioSet == false, "setScenario more than once");
    scenarioSet = true;
    
    inactiveAllSharedPins();
    switch (s) {
    case Scenario::UART : scenario_UART(); break;
    case Scenario::I2C : scenario_I2C(); break;
    case Scenario::SPI : scenario_SPI(); break;
    case Scenario::PWM : scenario_PWM(); break;
    case Scenario::DSHOT : scenario_DSHOT(); break;
    }
    chThdSleepMilliseconds(1);
  }


  bool isFirmwareMatchHardware()
  {
    int nbTestFailed = 0;
    
    BOARD_GROUP_DECLFOREACH(pin, DYNAMIC_FUNCTION_PIN) {
      palSetLineMode(pin, PAL_MODE_INPUT_PULLDOWN);
    }
    
    for (auto commander_line : (const ioline_t [])
	   {LINE_F0_a, LINE_F1_a, LINE_F2_a, LINE_F3_a, LINE_F4_a}) {
      palSetLineMode(commander_line, PAL_MODE_OUTPUT_PUSHPULL);
      palSetLine(commander_line);
    }
    chThdSleepMilliseconds(1);
    for (auto connected_line : (const ioline_t [])
	   {LINE_F0_b, LINE_F0_c,
	    LINE_F1_b, LINE_F1_c,
	    LINE_F2_b, LINE_F3_b, LINE_F4_b}) {
      if (palReadLine(connected_line) != PAL_HIGH) {
	//	DebugTrace("fail PAD %lu", PAL_PAD(connected_line));
	++nbTestFailed;
      }
    }
    //    DebugTrace("%d test failed", nbTestFailed);
    inactiveAllSharedPins();
    return nbTestFailed <= 2;
  }


}


namespace {
  void scenario_UART()
  {
    palSetLineMode(LINE_F0_b, PAL_MODE_ALTERNATE(F0_b_USART_AF));
    palSetLineMode(LINE_F1_c, PAL_MODE_ALTERNATE(F1_c_USART_AF));
    palSetLineMode(LINE_F2_b, PAL_MODE_ALTERNATE(F2_b_USART_AF));
    palSetLineMode(LINE_F3_b, PAL_MODE_ALTERNATE(F3_b_USART_AF));

  }
  
  void scenario_I2C()
  {
    palSetLineMode(LINE_PULLUP_SCL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode(LINE_PULLUP_SDA, PAL_MODE_OUTPUT_PUSHPULL);
    palSetLine(LINE_PULLUP_SCL);
    palSetLine(LINE_PULLUP_SDA);
    
    if(DynPin::i2cUnhangBus(&I2CD1) == false) {
      DebugTrace("unhang bus I2C1 failed");
    }
    palSetLineMode(LINE_F0_c, PAL_MODE_ALTERNATE(F0_c_I2C_AF) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetLineMode(LINE_F1_c, PAL_MODE_ALTERNATE(F1_c_I2C_AF) | PAL_STM32_OTYPE_OPENDRAIN);

#if  STM32_I2C_USE_I2C2
    if(DynPin::i2cUnhangBus(&I2CD2) == false) {
      DebugTrace("unhang bus I2C2 failed");
    }
    palSetLineMode(LINE_F3_b, PAL_MODE_ALTERNATE(F3_b_I2C_AF) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetLineMode(LINE_F4_b, PAL_MODE_ALTERNATE(F4_b_I2C_AF) | PAL_STM32_OTYPE_OPENDRAIN);
#endif
  }

  // SCK is on F3 (not F0)
  // Chip Select is GPIO Output managed by the driver : better managed by peripheral ?
  void scenario_SPI()
  {
    palSetLineMode(LINE_F1_a, PAL_MODE_ALTERNATE(F1_a_SPI_AF) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(LINE_F2_a, PAL_MODE_ALTERNATE(F2_a_SPI_AF) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(LINE_F3_a, PAL_MODE_ALTERNATE(F3_a_SPI_AF) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(LINE_F4_a, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

  }
  
  void scenario_PWM()
  {
    palSetLineMode(LINE_F1_b, PAL_MODE_ALTERNATE(F1_b_TIM_AF) | PAL_STM32_OSPEED_MID1);
    palSetLineMode(LINE_F2_b, PAL_MODE_ALTERNATE(F2_b_TIM_AF) | PAL_STM32_OSPEED_MID1);
    palSetLineMode(LINE_F3_b, PAL_MODE_ALTERNATE(F3_b_TIM_AF) | PAL_STM32_OSPEED_MID1);
    palSetLineMode(LINE_F4_b, PAL_MODE_ALTERNATE(F4_b_TIM_AF) | PAL_STM32_OSPEED_MID1);
  }
  
  void scenario_DSHOT()
  {
    palSetLineMode(LINE_F0_c, PAL_MODE_ALTERNATE(F0_c_USART_AF));
    palSetLineMode(LINE_F1_b, PAL_MODE_ALTERNATE(F1_b_TIM_AF) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(LINE_F2_b, PAL_MODE_ALTERNATE(F2_b_TIM_AF) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(LINE_F3_b, PAL_MODE_ALTERNATE(F3_b_TIM_AF) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(LINE_F4_b, PAL_MODE_ALTERNATE(F4_b_TIM_AF) | PAL_STM32_OSPEED_HIGHEST);
  }



}

#else // Minican



namespace DynPin {
  bool isFirmwareMatchHardware()
  {
    gpio_config_t context; // RAII save and restore
    int nbTestFailed = 0;
    BOARD_GROUP_DECLFOREACH(pin, DYNAMIC_FUNCTION_PIN) {
      palSetLineMode(pin, PAL_MODE_INPUT_PULLDOWN);
    }
    
    for (auto commander_line : (const ioline_t [])
	   {LINE_F0_a, LINE_F1_a, LINE_F2_a, LINE_F3_a, LINE_F4_a}) {
      palSetLineMode(commander_line, PAL_MODE_OUTPUT_PUSHPULL);
      palSetLine(commander_line);
    }
    chThdSleepMilliseconds(1);
    for (auto connected_line : (const ioline_t [])
	   {LINE_F0_b, LINE_F0_c,
	    LINE_F1_b, LINE_F1_c,
	    LINE_F2_b, LINE_F3_b, LINE_F4_b}) {
      if (palReadLine(connected_line) == PAL_LOW) {
	++nbTestFailed;
      }
    }
    BOARD_GROUP_FOREACH(pin, DYNAMIC_FUNCTION_PIN) {
      palSetLineMode(pin, PAL_MODE_INPUT_PULLUP);
    }
    
    for (auto commander_line : (const ioline_t [])
	   {LINE_F0_a, LINE_F1_a, LINE_F2_a, LINE_F3_a, LINE_F4_a}) {
      palClearLine(commander_line);
    }
    chThdSleepMicroseconds(10);
    for (auto connected_line : (const ioline_t [])
	   {LINE_F0_b, LINE_F0_c,
	    LINE_F1_b, LINE_F1_c,
	    LINE_F2_b, LINE_F3_b, LINE_F4_b}) {
      if (palReadLine(connected_line) == PAL_HIGH) {
	++nbTestFailed;
      }
    }

    const bool microCanHardware = nbTestFailed <= 2;
    const bool hardwareMatch = not microCanHardware;
    if (hardwareMatch) {
      context.restore();
    }
    return hardwareMatch;
  }

}
#endif
