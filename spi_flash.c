/**
  *@file spi_flash.c
  *@brief Driver for generic spi flash
  *@author Jason Berger
  *@date 03/14/2019
  *
  *@Datasheet http://www.adestotech.com/wp-content/uploads/DS-AT25SF041_044.pdf
  */

#include "spi_flash.h"


//Op codes
#define SPI_FLASH_OP_READ 0x03
#define SPI_FLASH_OP_WRITE 0x02
#define SPI_FLASH_OP_WR_ENABLE 0x06
#define SPI_FLASH_OP_CHIP_ERASE 0x60
#define SPI_FLASH_OP_STAT1 0x05
#define SPI_FLASH_OP_STAT2 0x35

//register flags
#define STAT1_BUSY_FLAG 0x01

//Macro for setting read/write address
#define SET_ADDR(op,addr,pAddr) pAddr[0] = op; pAddr[1] = addr >> 16; pAddr[2] = addr>>8; pAddr[3] = addr;


mrt_status_t spi_flash_init(spi_flash_t* dev, spi_flash_hw_cfg_t* hw, uint32_t memsize, uint32_t pagesize )
{
  memcpy(&dev->mHW, &hw, sizeof(spi_flash_hw_cfg_t));
  dev->mPageSize = pagesize;
  dev->mMemorySize = memsize;
  dev->mStat1 = 0;
  dev->mStat2 = 0;

  return MRT_STATUS_OK;
}


mrt_status_t spi_flash_erase_chip(spi_flash_t* dev)
{
	uint8_t op = FLASH_OP_CHIP_ERASE;
	uint8_t op_enable[2] = {0x01, 0x00};
  //erasing the chip requires two op codes for security

  //enable write
  spi_flash_enable_write(dev);

  //select chip , write op code and deselect
	MRT_GPIO_WRITE(dev->mHW.mCS, LOW);

	MRT_SPI_TRANSMIT(spi_handle, op_enable, 2, 50);

	MRT_GPIO_WRITE(dev->mHW.mCS, HIGH);

  //enable write
	spi_flash_enable_write(dev);

  //select chip , write op code and deselect
	MRT_GPIO_WRITE(dev->mHW.mCS, LOW);

	MRT_SPI_TRANSMIT(spi_handle, &op, 1, 50);

	MRT_GPIO_WRITE(dev->mHW.mCS, HIGH);

	spi_flash_wait(dev, 1000); //give a long timeout because it might be erasing a lot of data

  return MRT_STATUS_OK;
}


mrt_status_t spi_flash_read(spi_flash_t* dev, uint32_t addr, uint8_t* data, uint16_t len)
{
  uint8_t pAddr[4];
  MRT_GPIO_WRITE(dev->mHW.mCS, LOW);


	SET_ADDR(FLASH_OP_READ, addr,pAddr);
	MRT_SPI_TRANSMIT(spi_handle, pAddr, 4, 50);
	MRT_SPI_RECIEVE(spi_handle, data , len, 50);

	MRT_GPIO_WRITE(dev->mHW.mCS, HIGH);

  spi_flash_wait(dev,100);

  MRT_DELAY_MS(5);

  return MRT_STATUS_OK;
}


mrt_status_t spi_flash_write(spi_flash_t* dev, uint32_t addr, uint8_t* data, uint16_t len)
{
  uint8_t pAddr[4];
	uint32_t remPg = dev->mPageSize - (addr % dev->mPageSize);		//number of bytes remaining on current page
	uint16_t blockLen = len;
	uint16_t offset = 0;

	if(remPg < len)
	{
		blockLen = remPg;
	}

	spi_flash_enable_write(dev);

	MRT_GPIO_WRITE(dev->mHW.mCS, LOW);

	SET_ADDR(FLASH_OP_WRITE, addr, pAddr);
	MRT_SPI_TRANSMIT(spi_handle, pAddr, 4, 500);
	MRT_SPI_TRANSMIT(spi_handle, &data[offset] , blockLen, 500);

	MRT_GPIO_WRITE(dev->mHW.mCS, HIGH);

  spi_flash_wait(dev,100);
	MRT_DELAY_MS(5);

	len -=blockLen;
	addr += blockLen;
	offset+= blockLen;
	blockLen = dev->mPageSize;

	while(len > 0)
	{

		spi_flash_enable_write(dev);

		////printf("writing %d bytes to %d\n", blockLen, addr);
		MRT_GPIO_WRITE(dev->mHW.mCS, LOW

		SET_ADDR(FLASH_OP_WRITE, addr, pAddr);

		MRT_SPI_TRANSMIT(spi_handle, pAddr, 4, 100);
		MRT_SPI_TRANSMIT(spi_handle, &data[offset] , blockLen, 500);

		MRT_GPIO_WRITE(dev->mHW.mCS, HIGH);

		spi_flash_wait(dev,100);

		MRT_DELAY_MS(5);

		len -=blockLen;
		addr += blockLen;
		offset+= blockLen;
		if(blockLen > len)
			blockLen = len;
	}



  return MRT_STATUS_OK;
}

mrt_status_t spi_flash_enable_write(spi_flash_t* dev)
{
  uint8_t op_code = SPI_FLASH_OP_WR_ENABLE;

  //assert chip select
	MRT_GPIO_WRITE(dev->mHW.mCs, LOW);

  //write op code for enabling write
  MRT_SPI_TRANSMIT(dev->mHW.mSpi ,&op_code ,1, 500);

  //deassert chip select
	MRT_GPIO_WRITE(dev->mHW.mCs, HIGH);

  //wait for device to be ready with a 5ms timeout
	return spi_flash_wait(dev, 5);
}

mrt_status_t spi_flash_wait(spi_flash_t* dev, uint32_t timeout_ms)
{
  uint8_t op_code = SPI_FLASH_OP_STAT1;
  bool ready = false;

  //assert chip select
  MRT_GPIO_WRITE(dev->mHW.mCs, LOW);

  //write op code for enabling write
  MRT_SPI_TRANSMIT(dev->mHW.mSpi ,&op_code ,1, 10);


  while(timeout_ms > 0)
  {
    //read stat 1 register
    MRT_SPI_RECIEVE(dev->mHW.mSpi, &dev->mStat1, 1, 10);

    //check stat for busy flag
    if(dev->mStat1 & STAT1_BUSY_FLAG)
    {
      timeout_ms--;
    }
    else
    {
      ready = true;
      break;
    }
  }


  //deassert chip select
  MRT_GPIO_WRITE(dev->mHW.mCs, HIGH);

  if(ready)
    return MRT_STATUS_OK;
  else
    return MRT_STATUS_ERROR;
}
