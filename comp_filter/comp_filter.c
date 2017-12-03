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
#include <time.h>


// function declarations
void on_pause_pressed();
void on_pause_released();


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
	rc_imu_data_t imu_data; // imu data struct init
	rc_imu_config_t imu_conf = rc_default_imu_config();
	imu_conf.enable_magnetometer=1;
	float theta_a_raw;
	float theta_a;
	float theta_g;
	float theta_f;
	float theta_g_raw = 0;
	float last_theta_a_raw = 0;
	float last_theta_g_raw = 0;
	float last_theta_a = 0;
	float last_theta_g = 0;
	struct timeval c_time, last_time;
	gettimeofday(&last_time, NULL);
	float dt = 0;
	float time_tot = 0;
	float wc = .5;
	//record data
	FILE *f = fopen("data.txt", "w");
	if(rc_initialize_imu(&imu_data, imu_conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			// Green = running
			rc_set_led(GREEN, ON);
			rc_set_led(RED, OFF);
			//read imu data
			if(rc_read_gyro_data(&imu_data)<0){
				printf("read gyro failed\n");
			}
			if(rc_read_accel_data(&imu_data)<0){
				printf("read accel failed\n");
			}
			gettimeofday(&c_time, NULL);
			dt = (float)(c_time.tv_sec-last_time.tv_sec)+((c_time.tv_usec-last_time.tv_usec)/1000000.0);
			if(dt>100){
				dt = 0;
			}
			//calculate angle from acceleration data
			theta_a_raw = -atan2(imu_data.accel[2],imu_data.accel[1]);
			//calculate rotation from start with gyro data
			theta_g_raw = theta_g_raw + (float)dt*(imu_data.gyro[0]*DEG_TO_RAD) ;
			//apply low pass filter on accelerometer
			theta_a = (wc*dt*last_theta_a_raw)+((1-(wc*dt))*last_theta_a);
			//apply high pass filter on theta_g_raw
			theta_g = (1-(wc*dt))*last_theta_g + theta_g_raw - last_theta_g_raw;
			//get theta f
			theta_f = theta_a + theta_g;
			//print
			fprintf(f,"%.5f %.5f %.5f %.5f\r",time_tot,theta_a,theta_g,theta_f);
			printf("theta_a_raw: %.4f theta_a: %.4f theta_g_raw %.5f  theta_g %.5f  theta_f %.5f\r" , theta_a_raw,theta_a,theta_g_raw,theta_g,theta_f);
			//set last stuff
			last_time = c_time;
			last_theta_a = theta_a;
			last_theta_g = theta_g;
			last_theta_g_raw = theta_g_raw;
			last_theta_a_raw = theta_a_raw;
			time_tot = time_tot + dt;
		}
		else if(rc_get_state()==PAUSED){
			// do other things
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);
		}
		// always sleep at some point
		usleep(10000);

	}
	
	// exit cleanly
	rc_power_off_imu();
	rc_cleanup(); 
	fclose(f);
	return 0;
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
	
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}
