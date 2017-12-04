/*******************************************************************************
* config.h
*
* Contains the settings for configuration of rc_balance.c
*******************************************************************************/

#ifndef BALANCE_CONFIG
#define BALANCE_CONFIG

#define SAMPLE_RATE_HZ 100	// main filter and control loop speed
#define DT 0.01			// 1/sample_rate

// Structural properties of eduMiP
#define CAPE_MOUNT_ANGLE		-0.49 // increase if mip tends to roll forward
#define GEARBOX 				35.577
#define ENCODER_RES				60
#define WHEEL_RADIUS_M			0.034
#define TRACK_WIDTH_M			0.035
#define V_NOMINAL				7.4


// electrical hookups
#define MOTOR_CHANNEL_L			3
#define MOTOR_CHANNEL_R			2
#define MOTOR_POLARITY_L		1
#define MOTOR_POLARITY_R		-1
#define ENCODER_CHANNEL_L		3
#define ENCODER_CHANNEL_R		2
#define ENCODER_POLARITY_L		1
#define ENCODER_POLARITY_R		-1


// Thread Loop Rates
#define BATTERY_CHECK_HZ		5
#define SETPOINT_MANAGER_HZ		100
#define PRINTF_HZ				50

// other
#define TIP_ANGLE				0.85
#define START_ANGLE				0.3
#define START_DELAY				0.4
#define PICKUP_DETECTION_TIME	0.6
#define ENABLE_POSITION_HOLD	1
#define SOFT_START_SEC			0.7

#endif	//BALANCE_CONFIG
