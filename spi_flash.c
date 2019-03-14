/**
  *@file spi_flash.c
  *@brief Driver for generic spi flash
  *@author Jason Berger
  *@date 03/14/2019
  *
  *@Datasheet http://www.adestotech.com/wp-content/uploads/DS-AT25SF041_044.pdf
  */


mrt_status_t spi_flash_init(spi_flash_t* dev, spi_flash_hw_cfg_t* hw, uint32_t memsize, uint32_t pagesize )
{
  //TODO
  return MRT_STATUS_OK;
}


mrt_status_t spi_flash_erase_chip(spi_flash_t* dev)
{
  //TODO port
  writeEnable();
	uint8_t op = FLASH_OP_CHIP_ERASE;
	uint8_t op_enable[2] = {0x01, 0x00};

	SET_GPIO(cs_pin, LOW);
	HAL_SPI_Transmit(spi_handle, op_enable, 2, 500);
	SET_GPIO(cs_pin, HIGH);

	writeEnable();

	SET_GPIO(cs_pin, LOW);
	HAL_SPI_Transmit(spi_handle, &op, 1, 500);
	SET_GPIO(cs_pin, HIGH);



	wait();

  return MRT_STATUS_OK;
}


mrt_status_t spi_flash_read(spi_flash_t* dev, uint32_t addr, uint8_t* data, uint16_t len)
{
  //TODO port

  SET_GPIO(cs_pin, LOW);


	SET_ADDR(FLASH_OP_READ, addr);
	HAL_SPI_Transmit(spi_handle, pAddr, 4, 500);
	HAL_SPI_Receive(spi_handle, data , len, 500);
	SET_GPIO(cs_pin, HIGH);
	wait();
	HAL_Delay(5);
	return 0;

  return MRT_STATUS_OK;
}


mrt_status_t spi_flash_write(spi_flash_t* dev, uint32_t addr, uint8_t* data, uint16_t len)
{
  //TODO port
	uint32_t remPg = PAGE_SIZE - (addr % PAGE_SIZE);		//number of bytes remaining on current page
	uint16_t blockLen = len;
	uint16_t offset = 0;

	if(remPg < len)
	{
		blockLen = remPg;
	}

	writeEnable();

	SET_GPIO(cs_pin, LOW);

	////printf("writing %d bytes to %d\n", blockLen, addr);
	SET_ADDR(FLASH_OP_WRITE, addr);
	HAL_SPI_Transmit(spi_handle, pAddr, 4, 500);
	HAL_SPI_Transmit(spi_handle, &data[offset] , blockLen, 500);
	SET_GPIO(cs_pin, HIGH);
	wait();
	HAL_Delay(5);

	len -=blockLen;
	addr += blockLen;
	offset+= blockLen;
	blockLen = PAGE_SIZE;

	while(len > 0)
	{

		writeEnable();

		////printf("writing %d bytes to %d\n", blockLen, addr);
		SET_GPIO(cs_pin, LOW);
		SET_ADDR(FLASH_OP_WRITE, addr);
		HAL_SPI_Transmit(spi_handle, pAddr, 4, 100);
		HAL_SPI_Transmit(spi_handle, &data[offset] , blockLen, 500);
		SET_GPIO(cs_pin, HIGH);
		wait();
		HAL_Delay(5);

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
