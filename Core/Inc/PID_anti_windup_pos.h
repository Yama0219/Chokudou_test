//
// Created by kenke on 2023/01/20.
//

#ifndef PID_ANTI_WINDUP_POS_H
#define PID_ANTI_WINDUP_POS_H


#define Kp_pos 3.0f
#define Ki_pos 0.0f
#define Kd_pos 0.05f

#define MAX_OUT 950

float calc_output_PID_AW_pos(float target, float curr_val, float dt);

#endif //PID_ANTI_WINDUP_POS_H
