#include "gimbal_task.h"

#include "robot.h"
#include "remote.h"
#include "user_math.h"
#include "dji_motor.h"
#include "motor.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

static DJI_Motor_Handle_t *gimbal_yaw_motor;
static DJI_Motor_Handle_t *gimbal_pitch_motor;

void Gimbal_Task_Init()
{
    // Init Gimbal Hardware
    Motor_Config_t yaw_config = {
        .can_bus = 2,
        .speed_controller_id = 1,
        .offset = 0,
        .control_mode = POSITION_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .angle_pid = {
            .kp = 15.0f,
            .ki = 0.0f,
            .kd = 0.5f,
            .output_limit = GM6020_MAX_VOLTAGE,
            .integral_limit = 0.0f,
        },
    };
    
    Motor_Config_t pitch_config = {
        .can_bus = 2,
        .speed_controller_id = 2,
        .offset = 0,
        .control_mode = POSITION_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .angle_pid = {
            .kp = 15.0f,
            .ki = 0.0f,
            .kd = 0.5f,
            .output_limit = GM6020_MAX_VOLTAGE,
            .integral_limit = 0.0f,
        },
    };

    gimbal_yaw_motor = DJI_Motor_Init(&yaw_config, GM6020);
    gimbal_pitch_motor = DJI_Motor_Init(&pitch_config, GM6020);
}

void Gimbal_Ctrl_Loop()
{
    // Control loop for gimbal
    static float yaw_target_angle = 0.0f;
    static float pitch_target_angle = 0.0f;

    // Map remote input to target angle updates
    // Assuming right stick x for yaw, y for pitch
    // Sensitivity factor to convert stick value to angle change
    float sensitivity = 0.0001f; 
    
    yaw_target_angle += g_remote.controller.right_stick.x * sensitivity;
    pitch_target_angle += g_remote.controller.right_stick.y * sensitivity;

    // Apply constraints (principles from presentation - safety/limits)
    // Constrain pitch to avoid physical stops (example limits)
    if (pitch_target_angle > 0.5f) pitch_target_angle = 0.5f;
    if (pitch_target_angle < -0.5f) pitch_target_angle = -0.5f;

    // Yaw might wrap around or be continuous, usually continuous 0-2PI or infinite for GM6020
    // dji_motor driver seems to handle angles, but let's keep it simple for now

    DJI_Motor_Set_Angle(gimbal_yaw_motor, yaw_target_angle);
    DJI_Motor_Set_Angle(gimbal_pitch_motor, pitch_target_angle);
}