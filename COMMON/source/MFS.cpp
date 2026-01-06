#include "MFS.hpp"
#include "hal.h"
#include "hal_xsnor_stm_m95p.h"
#include "hardwareConf.hpp"
#include "hal_mfs.h"
#include "stdutil.h"
#include <array>
#include "etl/span.h"

namespace {
  hal_xsnor_stm_m95p_c m95p;
  xsnor_buffers_t      snor1buf __attribute__((section(DMA_SECTION), aligned(4)));
  mfs_nocache_buffer_t mfsbuf1  __attribute__((section(DMA_SECTION), aligned(4)));
  MFSDriver mfs1;
  
  const xsnor_config_t snorcfg_m95p = {
    .bus_type  = XSNOR_BUS_MODE_SPI,
    .bus = {
      .spi = {
	.drv   = &EepromSPID,
	.cfg   = &eepromSpiCfg
      }
    },
    .buffers    = &snor1buf,
    .options	= 0
  };


  const MFSConfig mfscfg1 = {
    .flashp           = (BaseFlash *)&m95p.fls,
    .erased           = 0xFFFFFFFFU,
    .bank_size        = MFS_BANK_SIZE_BYTES,
    .bank0_start      = BLANK_GAP_SIZE_SECTORS,
    .bank0_sectors    = MFS_BANK_SIZE_BYTES / SECTOR_SIZE_BYTES, 
    .bank1_start      = BLANK_GAP_SIZE_SECTORS + (MFS_BANK_SIZE_BYTES / SECTOR_SIZE_BYTES), 
    .bank1_sectors    = MFS_BANK_SIZE_BYTES / SECTOR_SIZE_BYTES, 
  };
}




namespace MFS {
   mfs_error_t start()
  {
    m95pObjectInit(&m95p);
    if (flash_error_t err = xsnorStart(&m95p, &snorcfg_m95p); err != FLASH_NO_ERROR)
      return MFS_ERR_FLASH_FAILURE;
    mfsObjectInit(&mfs1, &mfsbuf1);
    
    return mfsStart(&mfs1, &mfscfg1);
  }
  
  void stop()
  {
    mfsStop(&mfs1);
  }

   bool write(size_t recordIndex, const void *data, size_t size)
   {
     recordIndex++;
     const mfs_error_t err = mfsWriteRecord(&mfs1, recordIndex, size,
					    static_cast<const uint8_t *>(data));
     return (err == MFS_NO_ERROR);
   }

  bool  read(size_t recordIndex, void *data, size_t& size)
   {
     recordIndex++;
     const mfs_error_t err = mfsReadRecord(&mfs1, recordIndex, &size,
					   static_cast<uint8_t *>(data));
     return (err == MFS_NO_ERROR);
   }

  size_t  getlen()
  {
    uint32_t low = 1;
    uint32_t high = MFS_CFG_MAX_RECORDS;
    ssize_t result = 0;
    
    std::array<uint8_t, 4> dummy;
    
    while (low < high) {
      uint32_t mid = low + (high - low) / 2;
      size_t size = dummy.size();
      int err = mfsReadRecord(&mfs1, mid, &size, dummy.data());
      
      if (err == MFS_ERR_NOT_FOUND) {
	high = mid; // mid est invalide
      } else if ((err == MFS_NO_ERROR) || (err == MFS_ERR_INV_SIZE)) {
	result = static_cast<ssize_t>(mid); // mid est valide
	low = mid + 1;
      } else {
	return 0;
      }
    }
    
    return result; // 0 si rien trouvé
  }

  
   bool		eraseAll()
   {
     return mfsErase(&mfs1);
   }

  Eeprom_M95::Device *getDevice()
  {
    return static_cast<Eeprom_M95::Device *>(m95p.device.device);
  }
  
}
