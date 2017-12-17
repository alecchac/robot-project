/*******************************************************************************
* rc_project_template.c
*
* This is meant to be a skeleton program for robotics cape projects. 
* Change this description and file name 
*******************************************************************************/

// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>
#include "config.h"

//Define armed or disarmed to indicate if controller is running
typedef enum arm_state_t{
	ARMED,
	DISARMED
}arm_state_t;

typedef struct core_state_t{
	float wheelAngle;
	float theta;
	float phi;
	float vBatt;
}core_state_t;

//threads
void* battery_checker(void* ptr);
void* print_info(void* ptr);
void* outer_loop(void* ptr);

// function declarations
void on_pause_pressed();
void on_pause_released();
void comp_filter(float* theta_a,float* theta_g, float* theta_current);
int disarm_controller();
int arm_controller();
int reset_values();
int wait_for_starting_condition();

//global variables 
static float theta_a=0.0f;
static float theta_g=0.0f;
static float theta_ref = 0.0f;
static float sat_counter =0;
static arm_state_t arm_state;
rc_imu_data_t imu_data; // imu data struct init
core_state_t robot_info;
// loop variables
rc_ringbuf_t e_buf;
rc_ringbuf_t u_buf;
static float e_theta = 0;
static float last_e_theta_1=0;
static float last_e_theta_2=0;
static float u = 0;
static float last_u_1 = 0;
static float last_u_2 = 0;
static float last_e_phi = 0;
static float last_theta_ref = 0;
static float phi_current = 0;
static float e_phi = 0;
static float soft_start = 0;



/*******************************************************************************
* int main() 
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(){
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	// do your own initialization here
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);
	//init variables
	rc_alloc_ringbuf(&e_buf,3);
	rc_alloc_ringbuf(&u_buf,2);
	arm_state = DISARMED;
	rc_imu_config_t imu_conf = rc_default_imu_config(); //use default config
	imu_conf.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_conf.orientation = ORIENTATION_Y_UP;


	//----------------Done Init Variables---------------------------
	
	//-----------Init Inner Loop Function for IMU interrupt at 100Hz---------
	void inner_loop(){
		//comp_filter(&theta_a,&theta_g,&robot_info.theta);
		robot_info.theta = imu_data.dmp_TaitBryan[TB_PITCH_X] + CAPE_MOUNT_ANGLE; 
		float K =1.08*(V_NOMINAL/robot_info.vBatt);
		float saturation_limit = .9;
		float D1_num[] = D1_NUM;
		float D1_den[] = D1_DEN;
		if(arm_state == ARMED){
			e_theta = theta_ref - robot_info.theta;
			rc_insert_new_ringbuf_value(&e_buf,e_theta);
			last_e_theta_1 = rc_get_ringbuf_value(&e_buf,1);
			last_e_theta_2 = rc_get_ringbuf_value(&e_buf,2);
			last_u_1 = rc_get_ringbuf_value(&u_buf,0);
			last_u_2 = rc_get_ringbuf_value(&u_buf,1);
			u = K*((D1_num[0]*e_theta)+(D1_num[1]*last_e_theta_1)+(D1_num[2]*last_e_theta_2)-(D1_den[1]*last_u_1)\
				-(D1_den[2]*last_u_2));
			rc_insert_new_ringbuf_value(&u_buf,u);
			rc_set_motor(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * u); 
			rc_set_motor(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * u);
			if (u>=saturation_limit){
				u = saturation_limit;
				sat_counter += .01;
			}
			if (u<=-saturation_limit){
				u = -saturation_limit;
				sat_counter += .01;
			}
			if(sat_counter>D1_SATURATION_TIMEOUT){
				arm_state=DISARMED;
				disarm_controller();
			}
			
		}
	}
	//start battery thread
	pthread_t battery_thread;
	pthread_create(&battery_thread,NULL,battery_checker,(void*)NULL);
	//start print thread
	pthread_t print_thread;
	pthread_create(&print_thread,NULL,print_info,(void*)NULL);

	while(robot_info.vBatt ==0) rc_usleep(1000);

	//init outer loop thread
	pthread_t outer_loop_thread;
	pthread_create(&outer_loop_thread,NULL,outer_loop,(void*)NULL);

	//-------------------init imu stuff--------------------------------
	//inititalize imu
	if(rc_initialize_imu(&imu_data, imu_conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}
	//initialize dmp
	if(rc_initialize_imu_dmp(&imu_data,imu_conf)){
		printf("dmp init failed");
		return -1;
	}
	//set dmp interrupt fxn
	rc_set_imu_interrupt_func(&inner_loop);


	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 
	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			// do things
			rc_set_led(GREEN, ON);
			rc_set_led(RED, OFF);
			// if we got here the state is RUNNING, but controller is not
			// necessarily armed. If DISARMED, wait for the user to pick MIP up
			// which will we detected by wait_for_starting_condition()
			if(arm_state == DISARMED){
				if(wait_for_starting_condition()==0){
					arm_controller();
				} 
				else continue;
			}
		}
		else if(rc_get_state()==PAUSED){
			// do other things
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);
		}
		// always sleep at some point
		rc_usleep(10000);
	}
	
	// exit cleanly
	pthread_join(battery_thread,NULL);
	pthread_join(print_thread,NULL);
	pthread_join(outer_loop_thread,NULL);
	rc_power_off_imu();
	rc_cleanup();
	rc_disable_motors(); 
	return 0;
}

void* outer_loop(void* ptr){
	float phi_ref = 0.0f;
	float D2_num[] = D2_NUM;
	float D2_den[] = D2_DEN;
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			float wheelAngleR = (rc_get_encoder_pos(ENCODER_CHANNEL_R) * TWO_PI) \
								/(ENCODER_POLARITY_R * GEARBOX * ENCODER_RES);
			float wheelAngleL = (rc_get_encoder_pos(ENCODER_CHANNEL_L) * TWO_PI) \
								/(ENCODER_POLARITY_L * GEARBOX * ENCODER_RES);
			phi_current = (wheelAngleR) + robot_info.theta;
			e_phi = phi_ref-phi_current;
			theta_ref = D2_P*((D2_num[0]*e_phi) + (D2_num[1]*last_e_phi)-(D2_den[1]*last_theta_ref));
			//set last variables
			last_theta_ref = theta_ref;
			last_e_phi = e_phi;
			rc_usleep(50000);
		}
	}
	return NULL;
}
 


/*******************************************************************************
* void on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING)		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/*******************************************************************************
* void on_pause_pressed() 
*
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	rc_disable_motors();
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}

void comp_filter(float* theta_a,float* theta_g, float* theta_current){
	//define variables
	float dt = .01;
	float wc = FILTER_WC;
	float theta_a_raw=0;
	float theta_g_raw = 0;
	static float last_theta_a_raw = 0;
	static float last_theta_g_raw = 0;
	static float last_theta_a = 0;
	static float last_theta_g = 0;
	//calculate angle from acceleration data
	theta_a_raw = -atan2(imu_data.accel[2],imu_data.accel[1]);
	//calculate rotation from start with gyro data
	theta_g_raw = theta_g_raw + (float)dt*(imu_data.gyro[0]*DEG_TO_RAD) ;
	//apply low pass filter on accelerometer
	*theta_a = (wc*dt*last_theta_a_raw)+((1-(wc*dt))*last_theta_a);
	//apply high pass filter on theta_g_raw
	*theta_g = (1-(wc*dt))*last_theta_g + theta_g_raw - last_theta_g_raw;
	//get theta f
	*theta_current = *theta_a + *theta_g + CAPE_MOUNT_ANGLE;

	//set last stuff
	last_theta_a = *theta_a;
	last_theta_g = *theta_g;
	last_theta_g_raw = theta_g_raw;
	last_theta_a_raw = theta_a_raw;
} 

int disarm_controller(){
	rc_disable_motors();
	arm_state = DISARMED;
	return 0;
}

int arm_controller(){
	soft_start = 0;
	reset_values();
	rc_set_encoder_pos(ENCODER_CHANNEL_R,0);
	rc_set_encoder_pos(ENCODER_CHANNEL_L,0);
	arm_state = ARMED;
	rc_enable_motors();
	return 0;
}

void* battery_checker(void* ptr){
	float new_v;
	while(rc_get_state()!=EXITING){
		new_v = rc_battery_voltage();
		// if the value doesn't make sense, use nominal voltage
		if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
		robot_info.vBatt = new_v;
		rc_usleep(1000000 / BATTERY_CHECK_HZ);
	}
	return NULL;
}

// make a function to reset controller when armed
int reset_values(){
	rc_reset_ringbuf(&e_buf);
	rc_reset_ringbuf(&u_buf);
	last_e_theta_1 = 0.0f;
	last_e_theta_2 = 0.0f;
	last_u_1 = 0.0f;
	last_u_2 = 0.0f;
	u = 0.0f;
	theta_ref = 0.0f;
	sat_counter =0.0f;
	e_theta = 0.0f;
	phi_current = 0.0f;
	last_theta_ref = 0.0f;
	last_e_phi = 0.0f;
	return 0;
}


int wait_for_starting_condition(){
	int checks = 0;
	const int check_hz = 20;	// check 20 times per second
	int checks_needed = round(START_DELAY*check_hz);
	int wait_us = 1000000/check_hz; 

	// wait for MiP to be tipped back or forward first
	// exit if state becomes paused or exiting
	while(rc_get_state()==RUNNING){
		// if within range, start counting
		if(fabs(robot_info.theta) > START_ANGLE) checks++;
		// fell out of range, restart counter
		else checks = 0;
		// waited long enough, return
		if(checks >= checks_needed) break;
		rc_usleep(wait_us);
	}
	// now wait for MiP to be upright
	checks = 0;
	// exit if state becomes paused or exiting
	while(rc_get_state()==RUNNING){
		// if within range, start counting
		if(fabs(robot_info.theta) < START_ANGLE) checks++;
		// fell out of range, restart counter
		else checks = 0;
		// waited long enough, return
		if(checks >= checks_needed) return 0;
		rc_usleep(wait_us);
	}
	return -1;
}

void* print_info(void *ptr){
	while(rc_get_state()!=EXITING){
		printf("u: %f  theta_ref: %f  theta: %f  e_phi: %f Vbatt: %f \n" ,u,theta_ref,robot_info.theta,e_phi,robot_info.vBatt);
		rc_usleep(10000);
	}
	return NULL;
}