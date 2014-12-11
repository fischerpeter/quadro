#include "stm32f4xx.h"
#include "core_cm4.h"
#include "system_stm32f4xx.h"

//uint8_t L3GD20_Read1Byte (uint8_t reg);

void LSM303DLHC_ReadAllAxes (uint8_t reg, int8_t data[]);
void LSM303DLHC_Init_Default(void);
void LSM303DLHC_Send1byte(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t LSM303DLHC_Read1Byte (uint8_t reg);
void LSM303DLHC_updateAngle();
void LSM303DLHC_getCalibration();
int16_t LSM303DLHC_GetInt(uint8_t low, uint8_t high);

//volatile float accXangle, accYangle, accZangle; //winkel (Rückgabewerte!)
//int16_t accXraw, accYraw, accZraw;

struct accdata {
	int16_t Xraw;
	float Xrate, Yrate, Zrate;
	float Xangle, Yangle, Zangle;
	int16_t calX, calY, calZ;
	int16_t Yreadout;
} acc;

#define ADDRESS_AM	0x19

//	Accelerometer
#define WHO_AM_I 0x0F
#define CTRL_REG1_A	0x20
#define CTRL_REG2_A	0x21
#define CTRL_REG3_A	0x22
#define CTRL_REG4_A	0x23
#define CTRL_REG5_A	0x24
#define CTRL_REG6_A	0x25
#define DATACAPTURE_A	0x26
#define STATUS_REG_A	0x27
#define OUT_X_L_A		0x28
#define OUT_X_H_A		0x29
#define OUT_Y_L_A		0x2A
#define OUT_Y_H_A		0x2B
#define OUT_Z_L_A		0x2C
#define OUT_Z_H_A		0x2D
#define FIFO_CTRL_REG_A	0x2E
#define FIFO_SRC_REG_A	0x2F
#define INT1_CFG_A	0x30
#define INT1_SRC_A	0x31
#define INT1_THS_A	0x32
#define INT1_DURATION_A	0x33
#define INT2_CFG_A	0x34
#define INT2_SRC_A	0x35
#define INT2_THS_A	0x36
#define INT2_DURATION_A	0x37
#define CLICK_CFG_A	0x38
#define CLICK_SRC_A	0x39
#define CLICK_THS_A	0x3A
#define  TIME_LIMIT_A	0x3B
#define  TIME_LATENCY_A	0x3C
#define TIME_WINDOW_A	0x3D

//   Magnetometer
#define CRA_REG_M	0x00
#define CRB_REG_M	0x01
#define MR_REG_M	0x02
#define OUT_X_L_M		0x04
#define OUT_X_H_M		0x03
#define OUT_Y_L_M		0x08
#define OUT_Y_H_M		0x07
#define OUT_Z_L_M		0x06
#define OUT_Z_H_M		0x05
#define SR_REG_M	0x09
#define IR_REG_M	0x0A	// ? Steht so im Datenblatt
//#define IR_REG_M	0x0B	// ?
//#define IR_REG_M	0x0C	// ?
#define TEMP_OUT_H_M	0x31
#define TEMP_OUT_L_M	0x32


//Abtastraten
#define ODR1            0x10  /* 1Hz output data rate */
#define ODR10           0x20  /* 10Hz output data rate */
#define ODR25           0x30  /* 25Hz output data rate */
#define ODR50           0x40  /* 50Hz output data rate */
#define ODR100          0x50  /* 100Hz output data rate */
#define ODR200          0x60  /* 200Hz output data rate */
#define ODR400          0x70  /* 400Hz output data rate */
#define ODR1250         0x90  /* 1250Hz output data rate */
