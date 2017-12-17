/*******************************************************************************
* config.h
*
* Contains the settings for configuration of rc_balance.c
*******************************************************************************/

#ifndef BALANCE_CONFIG
#define BALANCE_CONFIG

#define SAMPLE_RATE_HZ 100	// main filter and control loop speed
#define DT 0.01			// 1/sample_rate
#define FILTER_WC 2

// Structural properties of eduMiP
#define CAPE_MOUNT_ANGLE		0.32 // increase if mip tends to roll forward
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



/*
// // inner loop controller 100hz new
#define 	D1_GAIN					1
#define 	D1_NUM					{-3.211, 5.469, -2.327}
#define 	D1_DEN					{ 1.000, -1.572, 0.5724}
#define 	D1_SATURATION_TIMEOUT	.4
*/

/*
// // inner loop controller 100hz new
#define 	D1_GAIN					1
#define 	D1_NUM					{-4.945, 8.862, -3.967}
#define 	D1_DEN					{ 1.000, -1.481, 0.4812}
#define 	D1_SATURATION_TIMEOUT	0.4
*/

// // inner loop controller 100hz new
#define 	D1_GAIN					1
#define 	D1_NUM					{-4.789,8.095, -3.367}
#define 	D1_DEN					{ 1.000, -1.25, 0.25}
#define 	D1_SATURATION_TIMEOUT	.4

// outer loop controller new 100hz
#define 	D2_GAIN					1
#define 	D2_NUM					{0.2176,  -0.2059}
#define 	D2_DEN					{1,  -0.5685}
#define 	THETA_REF_MAX			0.68
#define     D2_P                       0.4


// Thread Loop Rates
#define BATTERY_CHECK_HZ		5

// other
#define TIP_ANGLE				0.85
#define START_ANGLE				0.3
#define START_DELAY				1
#define PICKUP_DETECTION_TIME	0.6
#define ENABLE_POSITION_HOLD	1
#define SOFT_START_SEC			0.7

#endif	//BALANCE_CONFIG
