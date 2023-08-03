#include "dht20_drv.h"
extern I2C_HandleTypeDef hi2c3;

const uint8_t DHT20_ADDRESS = (0x38 << 1);
float    _humidity;
float    _temperature;
float    _humOffset;
float    _tempOffset;
uint8_t  _status;
uint32_t _lastRequest;
uint32_t _lastRead;
uint8_t  _bits[7];





void DHT20_Custom_Init(void){
	  _temperature = 0;
	  _humidity    = 0;
	  _humOffset   = 0;
	  _tempOffset  = 0;
	  _status      = DHT20_OK;
	  _lastRequest = 0;
	  _lastRead    = 0;
}

uint8_t isConnected()
{
	if(HAL_I2C_Master_Transmit(&hi2c3, DHT20_ADDRESS, NULL, 0, HAL_MAX_DELAY) == HAL_OK)return 1;
	return 0;
}
uint8_t getAddress()
{
  return DHT20_ADDRESS;
}
uint8_t resetSensor()
{
  uint8_t count = 0;
  if ((readStatus() & 0x18) != 0x18)
  {
    count++;
    if (_resetRegister(0x1B)) count++;
    if (_resetRegister(0x1C)) count++;
    if (_resetRegister(0x1E)) count++;
    HAL_Delay(10);
  }
  return count;
}

uint8_t read()
{
  //  do not read to fast == more than once per second.
  if (HAL_GetTick() - _lastRead < 1000)
  {
    return DHT20_ERROR_LASTREAD;
  }

  uint8_t status = requestData();
  if (status < 0) return status;
  //  wait for measurement ready
  uint32_t start = HAL_GetTick();
  while (isMeasuring())
  {
    if (HAL_GetTick() - start >= 1000)
    {
      return DHT20_ERROR_READ_TIMEOUT;
    }
  }
  //  read the measurement
  status = readData();
  if (status != 0) return status;

  //  convert it to meaningful data
  return convert();
}


uint8_t requestData()
{
  //  reset sensor if needed.
  resetSensor();

  //  GET CONNECTION
  uint8_t buf[3] = {0xAC, 0x33, 0x00};
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, DHT20_ADDRESS, buf, 3, HAL_MAX_DELAY);
  _lastRequest = HAL_GetTick();
  if(status!= HAL_OK)return DHT20_ERROR_CONNECT;
  return DHT20_OK ;
}


uint8_t readData()
{
	//  GET DATA
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c3, DHT20_ADDRESS, _bits, 7, HAL_MAX_DELAY);
	uint8_t allZero = 1;
	for (int i = 0; i < 7; i++)
	{
		allZero = allZero && (_bits[i] == 0);
	}
	if (allZero) return DHT20_ERROR_BYTES_ALL_ZERO;
	_lastRead = HAL_GetTick();
	if(status == HAL_OK)return DHT20_OK;
	return DHT20_ERROR_CONNECT;
}

uint8_t convert()
{
  //  CONVERT AND STORE
  _status      = _bits[0];
  uint32_t raw = _bits[1];
  raw <<= 8;
  raw += _bits[2];
  raw <<= 4;
  raw += (_bits[3] >> 4);
  _humidity = raw * 9.5367431640625e-5;   // ==> / 1048576.0 * 100%;

  raw = (_bits[3] & 0x0F);
  raw <<= 8;
  raw += _bits[4];
  raw <<= 8;
  raw += _bits[5];
  _temperature = raw * 1.9073486328125e-4 - 50;  //  ==> / 1048576.0 * 200 - 50;

  //  TEST CHECKSUM
  uint8_t _crc = _crc8(_bits, 6);
  //  Serial.print(_crc, HEX);
  //  Serial.print("\t");
  //  Serial.println(_bits[6], HEX);
  if (_crc != _bits[6]) return DHT20_ERROR_CHECKSUM;

  return DHT20_OK;
}

float getHumidity()
{
  return _humidity + _humOffset;
};


float getTemperature()
{
  return _temperature + _tempOffset;
};


void setHumOffset(float offset)
{
  _humOffset = offset;
};


void setTempOffset(float offset)
{
  _tempOffset = offset;
};


float getHumOffset()
{
  return _humOffset;
};


float getTempOffset()
{
  return _tempOffset;
};


////////////////////////////////////////////////
//
//  STATUS
//
uint8_t readStatus()
{
	uint8_t buf;
	HAL_I2C_Master_Receive(&hi2c3, DHT20_ADDRESS, &buf, 1, HAL_MAX_DELAY);
	HAL_Delay(5);  //  needed to stabilize timing
	return buf;
}


uint8_t isCalibrated()
{
  if((readStatus() & 0x08) == 0x08)return 1;
  return 0;
}

uint8_t isMeasuring()
{
	  if((readStatus() & 0x80) == 0x80)return 1;
	  return 0;
}

uint8_t isIdle()
{
	 if((readStatus() & 0x80) == 0x00)return 1;
		  return 0;
}

uint8_t internalStatus()
{
  return _status;
};

////////////////////////////////////////////////
//
//  TIMING
//
uint32_t lastRead()
{
  return _lastRead;
};


uint32_t lastRequest()
{
  return _lastRequest;
};


////////////////////////////////////////////////
//
//  PRIVATE
//
uint8_t _crc8(uint8_t *ptr, uint8_t len)
{
  uint8_t crc = 0xFF;
  while(len--)
  {
    crc ^= *ptr++;
    for (uint8_t i = 0; i < 8; i++)
    {
      if (crc & 0x80)
      {
        crc <<= 1;
        crc ^= 0x31;
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}


//  Code based on demo code sent by www.aosong.com
//  no further documentation.
//  0x1B returned 18, 0, 4
//  0x1C returned 18, 65, 0
//  0x1E returned 18, 8, 0
//    18 seems to be status register
//    other values unknown.
uint8_t _resetRegister(uint8_t reg)
{
  uint8_t value[3];
  value[0] = reg;
  value[1] = 0x00;
  value[2] = 0x00;
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, DHT20_ADDRESS, value, 3, HAL_MAX_DELAY);

  if(status != HAL_OK)return DHT20_ERROR_CONNECT;
  HAL_Delay(5);


  uint8_t buf[3];
  HAL_I2C_Master_Receive(&hi2c3, DHT20_ADDRESS, buf, 3, HAL_MAX_DELAY);


  HAL_Delay(10);

  uint8_t value2[3];
    value2[0] = (0xB0 | reg);
    value2[1] = value[1];
    value2[2] = value[2];
    HAL_StatusTypeDef tStatus = HAL_I2C_Master_Transmit(&hi2c3, DHT20_ADDRESS, value2, 3, HAL_MAX_DELAY);

  if(tStatus != HAL_OK)return DHT20_ERROR_CONNECT;
  HAL_Delay(5);
  return 1;
}


