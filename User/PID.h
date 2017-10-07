#define Kp	1.0
#define Ki	2.0
#define Kd	4.0
#define Ts	0.01
#define	a		(Kp + Ki*Ts/2 + Kd/Ts)
#define	b		(-1*Kp + Ki*Ts/2 - 2*Kd/Ts)
#define	c		(Kd/Ts)

extern float update_PID(float error);
