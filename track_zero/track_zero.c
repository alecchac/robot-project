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
//#include <math.h>


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
	//Define gear ratio and number of slots
	float gear_ratio = 35.577;
	float slots = 15;
	//define counts/rev by following equation
	float counts_per_rev = gear_ratio*slots*4;
	//initialize phis
	float phi1 = 0;
	float phi2 = 0;
	//initialzlize references
	float ref_phi = 0;
	//int error
	float error = 0;
	//set motor pins
	int motor1 = 2;
	int motor2 = 3;

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			//enable motors
			rc_enable_motors();
			// do things
			rc_set_led(GREEN, ON);
			rc_set_led(RED, OFF);
			//convert encoder counts to Radians of rotation 
			phi1 = rc_get_encoder_pos(motor1)*(1/counts_per_rev)*2*PI;
			phi2 = rc_get_encoder_pos(motor2)*(1/counts_per_rev)*2*PI;
			//determine set position (multiply by -1 to match other wheel)
			ref_phi = -phi2;
			//define error
			error = ref_phi - phi1;
			//set motor speed to error of the reference and current divided by a constant
			rc_set_motor(motor1, (error/5));
			//print encoder positions
			printf("Phi1: %f      Phi2: %f\n",phi1,phi2);

		}
		else if(rc_get_state()==PAUSED){
			// do other things
			rc_disable_motors();
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);
		}
		// always sleep at some point
		usleep(100000);
	}
	
	// exit cleanly
	rc_cleanup(); 
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
