/**
  *@file spi_flash.h
  *@brief Driver for generic spi flash
  *@author Jason Berger
  *@date 03/14/2019
  *
  *@Datasheet http://www.adestotech.com/wp-content/uploads/DS-AT25SF041_044.pdf
  */


typedef struct{
  mrt_spi_handle_t mSpi;
  mrt_gpio_t mCS;
} spi_flash_hw_cfg_t;

typedef struct{
  uint32_t mPageSize;
  uint32_t mMemorySize;
  uint8_t mStat1;
  uint8_t mStat2;
  spi_flash_hw_cfg_t mHW;
}spi_flash_t;

/**
  *@brief initialize spi flash device
  *@param dev ptr to spi flash device
  *@param hw ptr to hardware config struct
  *@param memsize size of memory in bytes
  *@param pagesize size of internal memory page
  *@pre hw must be configured with mSpi and mCS
  *@return status
  */
mrt_status_t spi_flash_init(spi_flash_t* dev, spi_flash_hw_cfg_t* hw, uint32_t memsize, uint32_t pagesize );

/**
  *@brief erase entire chip
  *@param dev ptr to spi flash device
  *@return status
  */
mrt_status_t spi_flash_erase_chip(spi_flash_t* dev);

/**
  *@brief read in data from flash at given address
  *@param dev ptr to spi flash device
  *@param addr address in flash to start reading from
  *@param data ptr to store data being read
  *@param len number of bytes to read
  *@return status
  */
mrt_status_t spi_flash_read(spi_flash_t* dev, uint32_t addr, uint8_t* data, uint16_t len);

/**
  *@brief write data to device
  *@param dev ptr to spi flash device
  *@param addr address in flash to start writing to
  *@param data ptr to data being written
  *@param len number of bytes to write
  *@return status
  */
mrt_status_t spi_flash_write(spi_flash_t* dev, uint32_t addr, uint8_t* data, uint16_t len);

/**
  *@brief enables writing to the memory
  *@param dev ptr to spi flash device
  *@return status
  */
mrt_status_t spi_flash_enable_write(spi_flash_t* dev);

/**
  *@brief waits for the chip to be ready
  *@param dev ptr to spi flash device
  *@return status
  */
mrt_status_t spi_flash_wait(spi_flash_t* dev, uint32_t timeout_ms);

/**
  *@brief device test intended for initial hardware bringup. addr and len can be used to set a reserved testing area
  *@param dev ptr to device to test
  *@param addr address in memory to test
  *@param len len of data to test with
  *@return status of test
  */
mrt_status_t spi_flash_test(spi_flash_t* dev, uint32_t addr, uint16_t len);
