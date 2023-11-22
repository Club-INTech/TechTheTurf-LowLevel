/* Le fichier .c contient 
 -les implémentations des fonctions;
 -les implémentations des variables
*/

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "motor.h"


#define WRAP_VAL 256


struct motor motor_new(int pin_1, int pin_2) {
    int slice = pwm_gpio_to_slice_num(pin_1); 
    //Il faut que le pin_1 et le pin_2 soient sur le même slice, sinon on doit
    //renvoyer une erreur 

    assert(slice == pwm_gpio_to_slice_num(pin_2));

    //On crée un moteur de structure motor qu'on va venir indiquer tous les
    //paramètres comme le slice, le rapport cyclique, la direction de rotation,
    //et les pins 
    struct motor motor; 
    motor.slice = slice; 
    motor.rotation = LEFT; //left par défaut et on va venir changer 
    motor.cyclical_report = 0; //initialiser à 0


    gpio_set_function(pin_1, GPIO_FUNC_PWM);
    gpio_set_function(pin_2, GPIO_FUNC_PWM);

    pwm_set_wrap(slice, 256);
    
    return motor;
}


//Fonction qui définit le sens de rotation du moteur 

void motor_set_rotation( struct motor *motor, float vitesse_rotation) {
   //On tourne dans le sens LEFT si la fréquence de rotation est positive
   //et dans le sens RIGHT sinon 
   if (vitesse_rotation > 0) {
        motor->rotation = LEFT; 
   } 
   else {
       motor->rotation = RIGHT; 
   }

    //On donne une valeur au rapport cyclique en fonction de la vitesse de
    //rotation
    vitesse_rotation = fabsf(vitesse_rotation); 
    if (vitesse_rotation > 255) {
        motor->cyclical_report = 1;
    }
    else {
        motor->cyclical_report = vitesse_rotation / 256;        
    }
}


//Fonction qui met en marche le moteur 

void motor_dispatch(struct motor *motor) {
    int cyclical_report = motor->cyclical_report * (WRAP_VAL - 1);
    pwm_set_chan_level(motor->slice, PWM_CHAN_A, motor->rotation ? 0 : cyclical_report);
    pwm_set_chan_level(motor->slice, PWM_CHAN_B, motor->rotation ? cyclical_report : 0);
    pwm_set_enabled(motor->slice, true);

}

