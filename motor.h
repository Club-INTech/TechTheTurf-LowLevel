/*Ce fichier .h contient
-les déclarations de fonctions;
-les définitions de structures;
-les constantes;
-les commentaires décrivant l'utilisation des fonctions et des variables.*/


#ifndef _DEFINE_MOTOR_H
#define _DEFINE_MOTOR_H


//Définition des structures 
struct motor {
    int pins[2];
    int slice;
    int rotation; 
    float cyclical_report;
};

//Définition des constantes 
//Défintion des PINs 
#define PIN_1_MOTOR_LEFT 4
#define PIN_2_MOTOR_LEFT 5
#define PIN_1_MOTOR_RIGHT 6
#define PIN_2_MOTOR_RIGHT 7

//Défintion de LEFT et RIGHT comme étant 0 et 1 respectivement 
#define LEFT 0
#define RIGHT 1 
 
//Déclaration des fonctions:
struct motor motor_new(int pin_1, int pin_2); 
void motor_set_rotation( struct motor *motor, float vitesse_rotation); 
void motor_dispatch(struct motor *motor);

#endif //DEFINE_MOTOR_H 
