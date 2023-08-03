#ifndef DHT20_DRV_H
#define DHT20_DRV_H
#include "main.h"


#define DHT20_WIRE_TIME_OUT         250000    //  microseconds
#define DHT20_OK                             0
#define DHT20_ERROR_CHECKSUM                 10
#define DHT20_ERROR_CONNECT                  11
#define DHT20_MISSING_BYTES                  12
#define DHT20_ERROR_BYTES_ALL_ZERO           13
#define DHT20_ERROR_READ_TIMEOUT             14
#define DHT20_ERROR_LASTREAD                 15


uint8_t     isConnected();
uint8_t  getAddress();
uint8_t      requestData();
uint8_t      readData();
uint8_t      convert();
uint8_t      read();
float    getHumidity();
float    getTemperature();
void     setHumOffset(float );
void     setTempOffset(float );
float    getHumOffset();
float    getTempOffset();
uint8_t  readStatus();
uint8_t     isCalibrated();
uint8_t     isMeasuring();
uint8_t     isIdle();
uint8_t      internalStatus();
uint32_t lastRead();
uint32_t lastRequest();
uint8_t  resetSensor();
uint8_t  _crc8(uint8_t *ptr, uint8_t len);
uint8_t     _resetRegister(uint8_t reg);
void 	 DHT20_Custom_Init(void);


#endif
