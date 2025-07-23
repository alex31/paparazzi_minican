#include "mfsOnM95p.h"
#include "hal.h"
#include "hal_mfs.h"
#include "mfs_test_root.h"
#include "stdutil.h"

//#ifdef M95P_RAM_EMULATION
#define SECTOR_SIZE_BYTES 512U
#define MFS_BANK_SIZE_BYTES (64U * 1024U)
#define BLANK_GAP_SIZE_SECTORS 0U
/* #else */
/* #define SECTOR_SIZE_BYTES 512U */
/* #define MFS_BANK_SIZE_BYTES (512U * 1024U) */
/* #define BLANK_GAP_SIZE_SECTORS ((128U * 1024U) / SECTOR_SIZE_BYTES) */
/* #endif */

static const SPIConfig spicfg = {
  .circular       = false,
  .slave          = false,
  .data_cb        = nullptr,
  .error_cb       = nullptr,
  .ssline         = LINE_SPI_EEPROM_CS,
  .cr1            = SPI_CR1_BR_0, // 5.3 Mhz : BR_0 for 42Mhz
  .cr2            = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

static hal_xsnor_stm_m95p_c m95p;

static xsnor_buffers_t snor1buf __attribute__ ((section(DMA_SECTION), aligned(8)));

static const xsnor_config_t snorcfg_m95p = {
  .bus_type      = XSNOR_BUS_MODE_SPI,
  .bus.spi.drv   = &SPID1,
  .bus.spi.cfg   = &spicfg,
  .buffers       = &snor1buf
};

/* const MFSConfig mfscfg1 = { */
/*   .flashp           = (BaseFlash *)&m95p.fls, */
/*   .erased           = 0xFFFFFFFFU, */
/*   .bank_size        = 2048U, */
/*   .bank0_start      = 1024U, */
/*   .bank0_sectors    = 256U, */
/*   .bank1_start      = 1280U, */
/*   .bank1_sectors    = 256U */
/* }; */

const MFSConfig mfscfg1 = {
  .flashp           = (BaseFlash *)&m95p.fls,
  .erased           = 0xFFFFFFFFU,
  .bank_size        = MFS_BANK_SIZE_BYTES,
  .bank0_start      = BLANK_GAP_SIZE_SECTORS,
  .bank0_sectors    = MFS_BANK_SIZE_BYTES / SECTOR_SIZE_BYTES, 
  .bank1_start      = BLANK_GAP_SIZE_SECTORS + (MFS_BANK_SIZE_BYTES / SECTOR_SIZE_BYTES), 
  .bank1_sectors    = MFS_BANK_SIZE_BYTES / SECTOR_SIZE_BYTES, 
};


bool mfsOverM95pInit(void)
{
  m95pObjectInit(&m95p);
  return (xsnorStart(&m95p, &snorcfg_m95p) == FLASH_NO_ERROR);
}

hal_xsnor_stm_m95p_c* mfsOverM95pGetDevice(void)
{
  return &m95p;
}
