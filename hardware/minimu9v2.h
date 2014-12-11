void IMU_Init(void);
void vMinIMU9v2Task(void *pvParameters);

struct attitude {
	float roll; 	//x (rollen) beim flugzeug
	float pitch;	//y (rauf runter) beim flugzeug
	float yaw;		//z	(links rechts) beim flugzeug
	float Xrate, Yrate, Zrate;

} att;


