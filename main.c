#include "servo.h"
#include "motor.h"
#include "incremental_encoder.h"

#include  <stdio.h>
#include  <pico/stdlib.h>
#include <math.h>

float distance_between_encoders = 86.8e-3; 
float encoder_radius = 17e-3;  

float ticks_per_m;
float ticks_per_radian;

struct pid pid_distance;
struct pid pid_angle; 

float desired_distance = 0.2;
float desired_angle = 0;

float servo_pid_distance;
float servo_pid_angle;

struct motor motor_left;
struct motor motor_right;

void setup() {
    initialise_pid(&pid_distance, 10, 0.2, 0);   
    
    initialise_pid(&pid_angle, 10, 0.2, 0);
    
    motor_right = motor_new(26, 27); 
    motor_left = motor_new(21, 20); 

    ticks_per_m = 1024.f / (2 * M_PI * encoder_radius);
    ticks_per_radian = 1024.f / (2 * M_PI);

    stdio_init_all();
    
    attach_encoder();
}
    

void loop() {
    int left_ticks = get_coder_left();
    int right_ticks = get_coder_right();

    float current_distance = (left_ticks + right_ticks) / (2 * ticks_per_m);
    float current_angle = -(left_ticks - right_ticks)/(2 * distance_between_encoders * ticks_per_radian);
    
    printf("current distance: %f current angle: %f \n", current_distance, current_angle);

    //Faire le PID pour la distance
    
    servo_pid_distance = calculate_result(&pid_distance, current_distance, desired_distance);
    
    //Faire le PID pour la distance
    
    servo_pid_angle = calculate_result(&pid_angle, current_angle, desired_angle);
    
    //Les moteurs:

    float consigne_right =  (servo_pid_distance +  servo_pid_angle * distance_between_encoders);

    motor_set_rotation(&motor_right, consigne_right);
    
    float consigne_left = (servo_pid_distance -  servo_pid_angle * distance_between_encoders);

    motor_set_rotation(&motor_left, consigne_left);
    
    printf("motor left %f motor right %f", consigne_left, consigne_right);
    printf("left ticks %i right ticks %i \n", left_ticks, right_ticks); 
    
    motor_dispatch(&motor_right);
    motor_dispatch(&motor_left);
    

}


int main(void) {

    setup();

    while (1) {
        loop();
	busy_wait_us(10000);
    }

    return 0; 
}
