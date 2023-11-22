#ifndef _DEFINE_SERVO_H
#define _DEFINE_SERVO_H


#include <stdbool.h>

//Definition du pas de temps dt 
#define DT 0.01

//définition d'une structure qui représente le correcteur pid
struct pid {
    float Kp; 
    float Ki; 
    float Kd;
};

//Fonction de contrôle du PID 

void initialise_pid(struct pid *pid, float Kp, float Ki, float Kd);
float calculate_result(struct pid *pid, float current_position, float desired_position);    

#endif //DEFINE_SERVO_H 
