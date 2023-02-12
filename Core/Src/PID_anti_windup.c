//
// Created by kenke on 2023/01/20.
//

#include "PID_anti_windup.h"
#define __abs(a) (((a) > 0) ? (a) : (-1*a))
float pid_aw_e = 0;
float pid_aw_e_sum = 0;
float pid_aw_e_diff = 0;
float pid_aw_result = 0;
float pid_aw_e_pre = 0;
float pid_aw_e_pre_pre = 0;
float e_for_i = 0;
float e_for_i_pre = 0;
float result_pre = 0;

float calc_output_PID_AW(float target, float curr_val, float dt) {
  pid_aw_e = target-curr_val;

  if (__abs(result_pre) > MAX_OUT) {
    e_for_i = 0;
  } else {
    e_for_i = pid_aw_e;
  }

  pid_aw_e_sum += ((e_for_i + e_for_i_pre)*dt/2.0f); // trapezoid integral

  pid_aw_e_diff = (3.0f*pid_aw_e - 4.0f*pid_aw_e_pre + pid_aw_e_pre_pre) / (2.0f*dt); // differential

  pid_aw_result = Kp * pid_aw_e + (Ki * pid_aw_e_sum) + (Kd * pid_aw_e_diff);

  pid_aw_e_pre_pre = pid_aw_e_pre;
  pid_aw_e_pre = pid_aw_e;

  e_for_i_pre = e_for_i;
  result_pre = pid_aw_result;
  return pid_aw_result;
}