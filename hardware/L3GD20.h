#include "stm32f4xx.h"
#include "core_cm4.h"
#include "system_stm32f4xx.h"

void L3GD20_updateAngle(void);
uint8_t L3GD20_Read1Byte (uint8_t reg);
void L3GD20_ReadAllAxes (uint8_t reg, int8_t data[]);
void L3GD20_Init_Default(void);
int16_t L3GD20_GetInt(uint8_t low, uint8_t high);
void vL3GD20FreerunTask(void *pvParameters);
void L3GD20_setScale();
void L3GD20_Write1Byte(uint8_t REG, uint8_t value);
void getCalibration ();

//-----Rückgabewert

struct gyrdata {
	float Xrate, Yrate, Zrate;
	float Xangle, Yangle, Zangle;
	int16_t calX, calY, calZ;
	int16_t Yreadout;
} gyr;



#define ADDRESS_G	0x6B

#define WHO_AM_I 0x0F
#define CTRL_REG1	0x20
#define CTRL_REG2	0x21
#define CTRL_REG3	0x22
#define CTRL_REG4	0x23
#define CTRL_REG5	0x24
#define REFERENCE	0x25
#define DATACAPTURE	0x25
#define OUT_TEMP	0x26
#define STATUS_REG	0x27
#define OUT_X_L		0x28
#define OUT_X_H		0x29
#define OUT_Y_L		0x2A
#define OUT_Y_H		0x2B
#define OUT_Z_L		0x2C
#define OUT_Z_H		0x2D
#define FIFO_CTRL_REG	0x2E
#define FIFO_SRC_REG	0x2F
#define INT1_CFG	0x30
#define INT1_SRC	0x31
#define INT1_THS_XH	0x32
#define INT1_THS_XL	0x33
#define INT1_THS_YH	0x34
#define INT1_THS_YL	0x35
#define INT1_THS_ZH	0x36
#define INT1_THS_ZL	0x37
#define INT1_DURATION	0x38
