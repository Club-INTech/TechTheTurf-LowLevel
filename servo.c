#include "servo.h"
#include "incremental_encoder.h"
 
volatile float integral = 0.0; //initialise le terme intégral
volatile float previous_error = 0.0; //initialise l'erreur précédente 

void initialise_pid(struct pid *pid, float Kp, float Ki, float Kd){
    pid->Kp = Kp; 
    pid->Ki = Ki; 
    pid->Kd = Kd; 
}

float calculate_result(
        struct pid *pid, 
        float current_position,
        float desired_position) {

    //Calculer l'erreur 
    float error = desired_position - current_position; 
    
    //Calculer le terme proportionnel 
    float proportional = pid->Kp * error; 

    //Calculer le terme intégral 
    integral += pid->Ki * error * DT;

    //Calculer le terme dérivé
    float derivative = pid->Kd * (error - previous_error) / DT;

    //Commande de Contrôle
    float control_output = proportional + integral + derivative; 

    //Mise à jour de l'erreur 
    previous_error = error; 

    return control_output; 

    
}
