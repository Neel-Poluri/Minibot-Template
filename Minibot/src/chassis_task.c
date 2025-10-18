#include "chassis_task.h"

#include "motor.h"
#include "dji_motor.h"

#include "robot.h"
#include "remote.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

static DJI_Motor_Handle_t* chassis_motors[4];  // Array to store motor pointers
float chassis_rad;

void Chassis_Task_Init()
{
    // Init chassis hardware
    for (int i = 0; i < 4; i++) {
        Motor_Config_t chassis_config = {
            .can_bus = 1, // what can bus the motor is on
            .speed_controller_id = i + 1, // identifier for each motor (1-4)
            .offset = 0, // Initial offset of the motor (used for encoder)
            .control_mode = VELOCITY_CONTROL, // Control mode of the motor
            .motor_reversal = MOTOR_REVERSAL_NORMAL, // Direction of the motor
            .velocity_pid = // pid
                {
                    .kp = 500.0f,
                    .kd = 0.0f,
                    .kf = 0.0f,
                    .output_limit = M2006_MAX_CURRENT, // m2006 is the motor
                },
        };
        chassis_motors[i] = DJI_Motor_Init(&chassis_config, M2006); // Initializing motor
    }
}

void Chassis_Ctrl_Loop()
{
    // Control loop for the chassis
    DJI_Motor_Set_Velocity(chassis_motors[0], 1000.0f);
}