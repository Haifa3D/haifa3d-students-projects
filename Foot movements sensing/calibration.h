// calibration.h
#ifndef CALIBRATION_H
#ifdef __cplusplus
 extern "C" {
#endif
#define CALIBRATION_H
void force_calibration(int *absolute_max_front, int *max_avg_front, int *avg_front, int *absolute_max_back, int *max_avg_back,int *avg_back, int *absolute_max_right,
 int *max_avg_right,int *avg_right, int *absolute_max_left, int *max_avg_left,int *avg_left, int pin_front, int pin_back, int pin_right, int pin_left);
void single_sensor_force_calibration(int *max_force, int *avg_force, int pin_force);
void avg_samples(int *avg_front_force_samples,int *avg_back_force_samples,int *avg_left_force_samples,int *avg_right_force_samples,int pin_front,
int pin_back, int pin_left, int pin_right);
//void check(int *n);
extern void print_string(const char *message);
extern void print_int(const int num);
#ifdef __cplusplus
 }
#endif
#endif