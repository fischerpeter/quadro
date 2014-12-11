
#include "L3GD20.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"
#include "core_cm4.h"
#include "system_stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "i2c.h"
//#include "uart_dma.h"

#include <calculations.h>

#include "stdlib.h"

//----- Settings
float G_GAIN = 0.00875;     // bei 2000 dps ist der umrechnungsfactor 0.07 (siehe setScale), 250dps=0.00875, 500dps=0.0175
float DT = 0.01;		 // Periode (bei 100Hz ) zum Integrieren des Winkels


//						  76543210
#define GYR_CTRL_REG1	0b00001111	//7 F geht
#define GYR_CTRL_REG4	0b00000000	//0x20 war damans wies gegangen ist

//----- Calibration Values


void L3GD20_updateAngle(void)
{
	int8_t dataGyr[6];
	int16_t raw[3];
	int16_t smoothed[3];

	//*/----- Auslesen
	L3GD20_ReadAllAxes(OUT_X_L, &dataGyr[0]);
	raw[0] = L3GD20_GetInt(dataGyr[0], dataGyr[1]) -gyr.calX;
	raw[1] = L3GD20_GetInt(dataGyr[2], dataGyr[3]) -gyr.calY;
	raw[2] = L3GD20_GetInt(dataGyr[4], dataGyr[5]) -gyr.calZ;
	//*/

	/*/
	raw[0] = ((L3GD20_Read1Byte(OUT_X_H) << 8) | (L3GD20_Read1Byte(OUT_X_L)) ) - gyr.calX;
	raw[1] = ((L3GD20_Read1Byte(OUT_Y_H) << 8) | (L3GD20_Read1Byte(OUT_Y_L)) ) - gyr.calY;
	raw[2] = ((L3GD20_Read1Byte(OUT_Z_H) << 8) | (L3GD20_Read1Byte(OUT_Z_L)) )-  gyr.calZ;
	//*/

	//----- filtern
	Calc_smoothValues(raw, smoothed, 3);

	//gyr.Yreadout = raw[1]+gyr.calY;
	gyr.Yreadout = smoothed[1];
	gyr.Yreadout = raw[1];

	//*/----- Auf grad/s umrechnen
	gyr.Xrate = (float) smoothed[0] * G_GAIN;
	gyr.Yrate = (float) smoothed[1] * G_GAIN;
	gyr.Zrate = (float) smoothed[2] * G_GAIN;
	//*/


	/*/----- Auf grad/s umrechnen OHNE Filter
	gyr.Xrate = (float) raw[0] * G_GAIN;
	gyr.Yrate = (float) raw[1] * G_GAIN;
	gyr.Zrate = (float) raw[2] * G_GAIN;
	//*/

	//----- Auf Winkel aufsummieren
	gyr.Xangle += gyr.Xrate * DT;
	gyr.Yangle += gyr.Yrate * DT;
	gyr.Zangle += gyr.Zrate * DT;

}


void vL3GD20FreerunTask(void *pvParameters)
{

	const portTickType xDelay = 20 / portTICK_RATE_MS; //zahl ist die periode in ms
	portTickType xLastWakeTime;

	L3GD20_Init_Default();


	while(1)
    {
		xLastWakeTime = xTaskGetTickCount();

		L3GD20_updateAngle();


		vTaskDelayUntil( &xLastWakeTime, xDelay);
    }
}




void L3GD20_Init_Default() {

	//Init I2C1
	I2C1_Init();
	//Set Ayes
	L3GD20_Write1Byte(CTRL_REG1, GYR_CTRL_REG1);

	//Set Scale
	L3GD20_Write1Byte(CTRL_REG4, GYR_CTRL_REG4);


	getCalibration ();


}




uint8_t L3GD20_Read1Byte (uint8_t reg) {
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
    I2C_Send7bitAddress(I2C1, ADDRESS_G<<1, I2C_Direction_Transmitter);
	    //Wait for I2C1 EV6, check if slave has acknowledged Master transmitter
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	//SER_putChar1(0xFF);
	//I2C_Send7bitAddress(I2C1, reg, 1);
	I2C_SendData(I2C1, reg);
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(I2C1, ENABLE);    // Send STRAT condition a second time
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 

	I2C_Send7bitAddress(I2C1, ADDRESS_G<<1, I2C_Direction_Receiver);
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
void L3GD20_ReadAllAxes (uint8_t reg, int8_t data[]) {
	//uint8_t value = 0xFF;
	int num = 6;
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
    I2C_Send7bitAddress(I2C1, ADDRESS_G<<1, I2C_Direction_Transmitter);
	    //Wait for I2C1 EV6, check if slave has acknowledged Master transmitter
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	//SER_putChar1(0xFF);
	//I2C_Send7bitAddress(I2C1, reg, 1);
	I2C_SendData(I2C1, reg | 0x80 );
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(I2C1, ENABLE);    // Send STRAT condition a second time
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 

	I2C_Send7bitAddress(I2C1, ADDRESS_G<<1, I2C_Direction_Receiver);
		//Wait for I2C1 EV6, check if slave has acknowledged Master transmitter
	    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    
	while (num)
	{	
		if (num==1)
		{
		I2C_AcknowledgeConfig(I2C1, DISABLE);
    	I2C_GenerateSTOP(I2C1, ENABLE);
			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
		}
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
		data[num] = I2C_ReceiveData(I2C1);
		//data++;
		num--;
	}



	//Send slave Address for write
	
    
	
}	 


int16_t L3GD20_GetInt(uint8_t low, uint8_t high) {
	int16_t interrim=0;
	
	interrim = high<<8;
	return low | interrim;
}




void L3GD20_Write1Byte(uint8_t REG, uint8_t value) {


	//Wait until I2C1 is not busy any more
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

    //Re-enable acknowledge
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    //Send I2C1 START condition
    I2C_GenerateSTART(I2C1, ENABLE);
    	//Wait for I2C1 EV5 --> Slave has acknowledged start condition
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));


    //Send slave Address for write
    I2C_Send7bitAddress(I2C1, ADDRESS_G<<1, I2C_Direction_Transmitter);
    	//Wait for I2C1 EV6, check if slave has acknowledged Master transmitter
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));



	I2C_SendData(I2C1, REG);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(I2C1, value);  // Continuous update, 0x20 für 2000 dps,0x10 für 500dps
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(I2C1, ENABLE);

}

void getCalibration ()
{

	int32_t calXgyr1 = 0;
	int32_t calYgyr1 = 0;
	int32_t calZgyr1 = 0;



	//uint16_t calY = 0;

	int bevorStart = 0;
	int i;
	for (i=0; i<bevorStart; i++)
		{
			int8_t dataGyr[6];
			L3GD20_ReadAllAxes(OUT_X_L, &dataGyr[0]);

			vTaskDelay(1);
		}


	portTickType xDelay = 2 / portTICK_RATE_MS; //zahl ist die periode in ms
	int sampleNum = 300;
	//int i;
	for (i=0; i<sampleNum; i++)
	{
		//*/
		gyr.calX = ((L3GD20_Read1Byte(OUT_X_H) << 8) | (L3GD20_Read1Byte(OUT_X_L)) );
		gyr.calY = ((L3GD20_Read1Byte(OUT_Y_H) << 8) | (L3GD20_Read1Byte(OUT_Y_L)) );
		gyr.calZ = ((L3GD20_Read1Byte(OUT_Z_H) << 8) | (L3GD20_Read1Byte(OUT_Z_L)) );

		calXgyr1 += gyr.calX;
		calYgyr1 += gyr.calY;
		calZgyr1 += gyr.calZ;



		//*/
		/*/
		int8_t dataGyr[6];
		int16_t raw[3];
		//----- Auslesen
		L3GD20_ReadAllAxes(OUT_X_L, &dataGyr[0]);
		raw[0] = L3GD20_GetInt(dataGyr[0], dataGyr[1]);
		raw[1] = L3GD20_GetInt(dataGyr[2], dataGyr[3]);
		raw[2] = L3GD20_GetInt(dataGyr[4], dataGyr[5]);
		calXgyr1 += (int)raw[0];
		calYgyr1 += (int)raw[1];
		calZgyr1 += (int)raw[2];

		//*/

		vTaskDelay(xDelay);
	}
	gyr.calX=calXgyr1/sampleNum;
	gyr.calY=calYgyr1/sampleNum;
	gyr.calZ=calZgyr1/sampleNum;

	//gyr.calY = 129;
}

