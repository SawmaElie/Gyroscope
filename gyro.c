// gyro.c

#include "gyro.h"
#include <main.h>
#include <math.h>
#include "stm32f4xx_hal.h"

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
extern I2C_HandleTypeDef hi2c1;


void Error(){
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
}

void Success(){
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}

//------------Start of Gyroscope MEMS-------------------

uint8_t SPIx_WriteRead(uint8_t Byte){
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&Byte, (uint8_t *)&receivedbyte, 1, 0x1000) != HAL_OK){
		Error();
	}
	else Success();
	return receivedbyte;
}

void Gyro_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead){
	if(NumByteToRead > 0x01) ReadAddr |= (uint8_t) (READWRITE_CMD | MULTIPLEBYTE_CMD);
	else ReadAddr |= (uint8_t)READWRITE_CMD;
	CS_ON;
	SPIx_WriteRead(ReadAddr);
	while(NumByteToRead > 0x00){
		*pBuffer = SPIx_WriteRead(DUMMY_BYTE);
		NumByteToRead--;
		pBuffer++;
	}
	CS_OFF;
}

void Gyro_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite){
	CS_OFF;
	if(NumByteToWrite > 0x01) WriteAddr |= (uint8_t) MULTIPLEBYTE_CMD;
	CS_ON;
	SPIx_WriteRead(WriteAddr);
	while(NumByteToWrite >= 0x01){
		SPIx_WriteRead(*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}
	CS_OFF;
}

uint8_t Gyro_ReadID(void){
	uint8_t ctrl = 0;
	Gyro_IO_Read(&ctrl, 0x0F, 1);
	return ctrl;
}

void Gyro_Ini(void){
	uint8_t ctrl = 0x00;
	if (Gyro_ReadID() == 0xD4) {
		//CTRL_REG1
		//ODR 760 Hz; PD; Zen; Xen; Yen
		ctrl = 0b11111111;
		Gyro_IO_Write(&ctrl, 0x20, 1);
		//CTRL_REG2
		//High-pass filter cut off freq 0.09
		ctrl = 0b00101001;
		Gyro_IO_Write(&ctrl, 0x21, 1);
		//CTRL_REG3
		ctrl = 0b00001000;
		Gyro_IO_Write(&ctrl, 0x22, 1);
		//CTRL_REG4
		ctrl = 0b00010000;
		Gyro_IO_Write(&ctrl, 0x23, 1);
		//CTRL_REG5
		ctrl = 0b00010000;
		Gyro_IO_Write(&ctrl, 0x24, 1);
	}
	else Error();
	HAL_Delay(500);
}

void Gyro_GetXYZ(int16_t* pData){
	uint8_t buffer[6];
	float valueinfloat = 0;
	Gyro_IO_Read((uint8_t*)&buffer[0], 0x28, 1);
	Gyro_IO_Read((uint8_t*)&buffer[1], 0x29, 1);
	Gyro_IO_Read((uint8_t*)&buffer[2], 0x2A, 1);
	Gyro_IO_Read((uint8_t*)&buffer[3], 0x2B, 1);
	Gyro_IO_Read((uint8_t*)&buffer[4], 0x2C, 1);
	Gyro_IO_Read((uint8_t*)&buffer[5], 0x2D, 1);

	for(int i = 0; i < 3; i++){
		valueinfloat = ((buffer[2*i+1] << 8) + buffer[2*i]);
		pData[i] = (int16_t)valueinfloat;
	}
}

//------------end of Gyroscope MEMS-------------------

//-------------Start of Gyroscope GY-521-----------//


// Function to initialize MPU6050

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}

//---------Reading Data from the Gyro--------//

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

//-------------End of Gyroscope GY-521-----------//


