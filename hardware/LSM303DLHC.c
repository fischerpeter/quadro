#include "LSM303DLHC.h"

#include "stm32f4xx.h"
#include "core_cm4.h"
#include "system_stm32f4xx.h"
#include "i2c.h"
#include "stm32f4xx_i2c.h"
#include "led.h"
#include <math.h>

//----- Settings
			// Led ist an wenn LSM303DLHC bearbeitet wird
//-----constants
//float M_PI= 3.14159265358979323846;
//float M_PI= 3.1415926535897;
float RAD_TO_DEG = 57.29578;
//----- Calibration Values
//int16_t calX, calY, calZ;



void LSM303DLHC_updateAngle(void)
{

	int8_t dataAcc[6];
	int16_t raw[3];

	//*-----Lesen der Werte-----
	LSM303DLHC_ReadAllAxes(OUT_X_L_A, &dataAcc[0]);
	raw[0] = LSM303DLHC_GetInt(dataAcc[0], dataAcc[1]);
	raw[1] = LSM303DLHC_GetInt(dataAcc[2], dataAcc[3]);
	raw[2] = LSM303DLHC_GetInt(dataAcc[4], dataAcc[5]);
	//*/

	/*-----Lesen der Werte-----
	accXraw = (((LSM303DLHC_Read1Byte(OUT_X_H_A) << 8) | (LSM303DLHC_Read1Byte(OUT_X_L_A)) )>>4);// - calX;
	accYraw = (((LSM303DLHC_Read1Byte(OUT_Y_H_A) << 8) | (LSM303DLHC_Read1Byte(OUT_Y_L_A)) )>>4);// - calY;
	accZraw = (((LSM303DLHC_Read1Byte(OUT_Z_H_A) << 8) | (LSM303DLHC_Read1Byte(OUT_Z_L_A)) )>>4);// - calZ;
	//*/
	acc.Xraw = raw[0];

	/* ******************von http://ozzmaker.com/2013/04/29/guide-to-interfacing-a-gyro-and-accelerometer-with-a-raspberry-pi/

	//*/
	//-----Auf degree umrechnen-----
	acc.Xangle = (float) (atan2(raw[1],raw[2])+M_PI)*RAD_TO_DEG;
	acc.Yangle = (float) (atan2(raw[2],raw[0])+M_PI)*RAD_TO_DEG -90;
	//accXangle = (float) (atan2(accYraw,accZraw))*RAD_TO_DEG;
	//accYangle = (float) (atan2(accZraw,accXraw))*RAD_TO_DEG;

	//*/
	if (acc.Xangle >180)
	{
	          acc.Xangle -= (float)360.0;
	}
	         if (acc.Yangle >180)
	         acc.Yangle -= (float)360.0;

}

void LSM303DLHC_Init_Default() {

	LSM303DLHC_Send1byte(ADDRESS_AM ,CTRL_REG1_A, 0b01010111); // x57  d87
	LSM303DLHC_Send1byte(ADDRESS_AM, CTRL_REG4_A, 0b00101000); // x28  d40


	//---Calibration wird garnicht gebraucht! darf auch nicht, sonst sind alle werte 0 und die erdbeschleunigung ist weg
	LSM303DLHC_getCalibration();



}

void LSM303DLHC_Send1byte(uint8_t addr, uint8_t reg, uint8_t value) {

	//Wait until I2C1 is not busy any more
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
 
    //Re-enable acknowledge
    I2C_AcknowledgeConfig(I2C1, ENABLE);
 
    //Send I2C1 START condition
    I2C_GenerateSTART(I2C1, ENABLE);
    	//Wait for I2C1 EV5 --> Slave has acknowledged start condition
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
 
	
    //Send slave Address for write
    I2C_Send7bitAddress(I2C1, addr<<1, I2C_Direction_Transmitter);
    	//Wait for I2C1 EV6, check if slave has acknowledged Master transmitter
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	
	
	I2C_SendData(I2C1, reg);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(I2C1, value);
	//I2C_SendData(I2C1, 0x97); // LowPowerMode , All 3 Axes On, Normal(1.355kHz)/LowPowre(5.376 kHz)
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(I2C1, ENABLE);

}

void LSM303DLHC_ReadAllAxes (uint8_t reg, int8_t data[]) {
	//uint8_t value = 0xFF;
	//int num = 6;
	//Wait until I2C1 is not busy any more
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
 
    //Re-enable acknowledge
    I2C_AcknowledgeConfig(I2C1, ENABLE);
 
    //Send I2C1 START condition
    I2C_GenerateSTART(I2C1, ENABLE);
    	//Wait for I2C1 EV5 --> Slave has acknowledged start condition
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
 
	
    //Send slave Address for write
	//I2C_SendData(I2C1, 0xD7);
    I2C_Send7bitAddress(I2C1, ADDRESS_AM<<1, I2C_Direction_Transmitter);
	    //Wait for I2C1 EV6, check if slave has acknowledged Master transmitter
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	//SER_putChar1(0xFF);
	//I2C_Send7bitAddress(I2C1, reg, 1);
	I2C_SendData(I2C1, reg | 0x80 );		 //Register + MSb 1 (wegen 1=read)
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(I2C1, ENABLE);    // Send STRAT condition a second time
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 

	I2C_Send7bitAddress(I2C1, ADDRESS_AM<<1, I2C_Direction_Receiver);
		//Wait for I2C1 EV6, check if slave has acknowledged Master transmitter
	    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	data[0] = I2C_ReceiveData(I2C1);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	data[1] = I2C_ReceiveData(I2C1);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	data[2] = I2C_ReceiveData(I2C1);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	data[3] = I2C_ReceiveData(I2C1);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	data[4] = I2C_ReceiveData(I2C1);
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
	data[5] = I2C_ReceiveData(I2C1);	     

	
    
	
}	 

uint8_t LSM303DLHC_Read1Byte (uint8_t reg) {
	uint8_t value = 0xFF;
	//Wait until I2C1 is not busy any more
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
 
    //Re-enable acknowledge
    I2C_AcknowledgeConfig(I2C1, ENABLE);
 
    //Send I2C1 START condition
    I2C_GenerateSTART(I2C1, ENABLE);
    	//Wait for I2C1 EV5 --> Slave has acknowledged start condition
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
 
	
    //Send slave Address for write
	//I2C_SendData(I2C1, 0xD7);
    I2C_Send7bitAddress(I2C1, ADDRESS_AM<<1, I2C_Direction_Transmitter);
	    //Wait for I2C1 EV6, check if slave has acknowledged Master transmitter
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	//SER_putChar1(0xFF);
	//I2C_Send7bitAddress(I2C1, reg, 1);
	I2C_SendData(I2C1, reg);
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(I2C1, ENABLE);    // Send STRAT condition a second time
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 

	I2C_Send7bitAddress(I2C1, ADDRESS_AM<<1, I2C_Direction_Receiver);
		//Wait for I2C1 EV6, check if slave has acknowledged Master transmitter
	    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    //value = I2C_ReceiveData(I2C1);


	//Send slave Address for write
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	
    I2C_GenerateSTOP(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
    value = I2C_ReceiveData(I2C1);
	return value;
}

void LSM303DLHC_getCalibration ()
{

	int32_t calXacc1 = 0;
	int32_t calYacc1 = 0;
	int32_t calZacc1 = 0;



	//uint16_t calY = 0;

	int bevorStart = 0;
	int i;
	for (i=0; i<bevorStart; i++)
		{
			int8_t dataGyr[6];
			LSM303DLHC_ReadAllAxes(OUT_X_L_A, &dataGyr[0]);

			vTaskDelay(1);
		}


	portTickType xDelay = 2 / portTICK_RATE_MS; //zahl ist die periode in ms
	int sampleNum = 1000;
	//int i;
	for (i=0; i<sampleNum; i++)
	{
		//*/
		acc.calX = ((LSM303DLHC_Read1Byte(OUT_X_H_A) << 8) | (LSM303DLHC_Read1Byte(OUT_X_L_A)) );
		acc.calY = ((LSM303DLHC_Read1Byte(OUT_Y_H_A) << 8) | (LSM303DLHC_Read1Byte(OUT_Y_L_A)) );
		acc.calZ = ((LSM303DLHC_Read1Byte(OUT_Z_H_A) << 8) | (LSM303DLHC_Read1Byte(OUT_Z_L_A)) );

		calXacc1 += acc.calX;
		calYacc1 += acc.calY;
		calZacc1 += acc.calZ;

		vTaskDelay(xDelay);
	}
	acc.calX=calXacc1/sampleNum;
	acc.calY=calYacc1/sampleNum;
	acc.calZ=calZacc1/sampleNum;

	//gyr.calY = 129;
}

int16_t LSM303DLHC_GetInt(uint8_t low, uint8_t high) {
	int16_t interrim=0;

	interrim = high<<8;
	return low | interrim;
}
