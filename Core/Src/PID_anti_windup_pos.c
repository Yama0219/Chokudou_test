//
// Created by kenke on 2023/01/20.
//

#include "PID_anti_windup_pos.h"
#define __abs(a) (((a) > 0) ? (a) : (-1*a))

float pid_aw_e_pos = 0;
float pid_aw_e_sum_pos = 0;
float pid_aw_e_diff_pos = 0;
float pid_aw_result_pos = 0;
float pid_aw_e_pre_pos = 0;
float pid_aw_e_pre_pre_pos = 0;
float e_for_i_pos = 0;
float e_for_i_pre_pos = 0;
float result_pre_pos = 0;

float calc_output_PID_AW_pos(float target, float curr_val, float dt) {
  pid_aw_e_pos = target-curr_val;

  if (__abs(result_pre_pos) > MAX_OUT) {
    e_for_i_pos = 0;
  } else {
    e_for_i_pos = pid_aw_e_pos;
  }

  pid_aw_e_sum_pos += ((e_for_i_pos + e_for_i_pre_pos)*dt/2.0f); // trapezoid integral

  pid_aw_e_diff_pos = (3.0f*pid_aw_e_pos - 4.0f*pid_aw_e_pre_pos + pid_aw_e_pre_pre_pos) / (2.0f*dt); // differential

  pid_aw_result_pos = Kp_pos * pid_aw_e_pos + (Ki_pos * pid_aw_e_sum_pos) + (Kd_pos * pid_aw_e_diff_pos);

  pid_aw_e_pre_pre_pos = pid_aw_e_pre_pos;
  pid_aw_e_pre_pos = pid_aw_e_pos;

  e_for_i_pre_pos = e_for_i_pos;
  result_pre_pos = pid_aw_result_pos;
  return pid_aw_result_pos;
}