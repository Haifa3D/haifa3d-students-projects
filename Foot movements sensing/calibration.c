// calibration.c
#include "calibration.h"
#include <Arduino.h>
#define MAX_SAMPLES 100
extern void print_string(const char *message);
extern void print_int(const int num);
void force_calibration(int *absolute_max_front, int *max_avg_front, int *avg_front, int *absolute_max_back, int *max_avg_back,int *avg_back, int *absolute_max_right,
 int *max_avg_right,int *avg_right, int *absolute_max_left, int *max_avg_left,int *avg_left, int pin_front, int pin_back, int pin_right, int pin_left){
    char *str = "started calibration stand normally \n ";
    print_string(str);
    delay(1000);
    single_sensor_force_calibration(absolute_max_front, avg_front, pin_front);
    single_sensor_force_calibration(absolute_max_back, avg_back, pin_back);
    single_sensor_force_calibration(absolute_max_right, avg_right, pin_right);
    single_sensor_force_calibration(absolute_max_left, avg_left, pin_left);
    delay(2000);
    str = "lean forward \n ";
    print_string(str);
    delay(2000);
    single_sensor_force_calibration(absolute_max_front, max_avg_front, pin_front);
    delay(2000);
    str = "Now lean backwards \n ";
    print_string(str);
    delay(2000);
    single_sensor_force_calibration(absolute_max_back, max_avg_back, pin_back);
    delay(2000);
    str = "Now lean right \n ";
    print_string(str);
    delay(2000);
    single_sensor_force_calibration(absolute_max_right, max_avg_right, pin_right);
    delay(2000);
    str = "Now lean left \n ";
    print_string(str);
    delay(2000);
    single_sensor_force_calibration(absolute_max_left, max_avg_left, pin_left);
    delay(2000);

 }
void single_sensor_force_calibration(int *max_force, int *avg_force, int pin_force) {
    int force_value = analogRead(pin_force);
    int calibration_samples = 0;
    int calibration_tot = 0;
    while(calibration_samples < 1000){
        //check the values of the sensors
        calibration_samples +=1;
        force_value = analogRead(pin_force);
        calibration_tot += force_value;

        if (force_value > *max_force){
            *max_force=force_value;
        }
        delay(1);
    }
    *avg_force=calibration_tot/calibration_samples;
}


void avg_samples(int *avg_front_force_samples,int *avg_back_force_samples,int *avg_left_force_samples,int *avg_right_force_samples,int pin_front,
int pin_back, int pin_left, int pin_right){
  int num_of_samples = 0;
  int force_value_back = 0;
  int force_value_front = 0;
  int force_value_right = 0;
  int force_value_left = 0;

  //initialize the sum and the average of the samples
  int sum_front_force_samples = 0;
  int sum_back_force_samples = 0;
  int sum_right_force_samples = 0;
  int sum_left_force_samples = 0;

  //calculate the average of MAX_SAMPLES samples to normilize the errors
  while(num_of_samples < MAX_SAMPLES){
    num_of_samples +=1;
    force_value_back = analogRead(pin_back);
    force_value_front = analogRead(pin_front);
    force_value_right = analogRead(pin_right);
    force_value_left = analogRead(pin_left);
    sum_front_force_samples += force_value_front;
    sum_back_force_samples += force_value_back;
    sum_right_force_samples += force_value_right;
    sum_left_force_samples += force_value_left;
    delay(1);
  }
  *avg_front_force_samples = sum_front_force_samples / num_of_samples;
  *avg_back_force_samples = sum_back_force_samples / num_of_samples;
  *avg_right_force_samples = sum_right_force_samples / num_of_samples;
  *avg_left_force_samples = sum_left_force_samples / num_of_samples;
}



