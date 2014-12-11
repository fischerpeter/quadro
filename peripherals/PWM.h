

int pwm_ch1;		// wert zwischen 0 und 100 (quasi % von pulsebase)
int pwm_ch2;		// vom hauptprogram setzten
int pwm_ch3;
int pwm_ch4;

//settings

#define periode  20000 //periodendauer in us 25000=25ms
#define pulsemax 2000	//dauer des max impulses ()
#define pulsemin 1000	//dauer des max impulses ()


void vPWMTask(void *pvParameters);
void Timer1_PWMinit(void);
