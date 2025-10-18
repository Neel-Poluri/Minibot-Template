#include "chassis_task.h"

#include "motor.h"
#include "dji_motor.h"
#include <math.h>

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
    
    // Get desired chassis velocities from remote control or autonomous system
    // These represent the desired motion in the chassis frame
    float vx = 0.0f;  // Forward/backward velocity (m/s)
    float vy = 0.0f;  // Left/right velocity (m/s)
    float omega = 0.0f;  // Rotational velocity (rad/s)
    
    // TODO: Get actual command inputs from remote control
    // Example: vx = g_remote.chassis_vx_command;
    
    // Apply kinematic transformation to convert chassis velocities to wheel speeds
    // For omni-wheel drivetrain with 45° wheel mounting
    // This is the kinematic mapping from the presentation
    float wheel_speeds[4];
    
    // Kinematic equations for 45° omni wheels (from presentation slides 12-13)
    wheel_speeds[0] = (vx - vy - omega * chassis_rad);  // Front-left wheel
    wheel_speeds[1] = (vx + vy + omega * chassis_rad);  // Front-right wheel  
    wheel_speeds[2] = (vx + vy - omega * chassis_rad);  // Back-left wheel
    wheel_speeds[3] = (vx - vy + omega * chassis_rad);  // Back-right wheel
    
    // Apply desaturation (slide 22) - scale down if any wheel exceeds max speed
    float max_speed = 1000.0f;  // Maximum wheel speed in RPM
    float max_calculated_speed = 0.0f;
    
    // Find the maximum calculated speed
    for (int i = 0; i < 4; i++) {
        if (fabsf(wheel_speeds[i]) > max_calculated_speed) {
            max_calculated_speed = fabsf(wheel_speeds[i]);
        }
    }
    
    // Scale down proportionally if needed
    if (max_calculated_speed > max_speed) {
        float scale_factor = max_speed / max_calculated_speed;
        for (int i = 0; i < 4; i++) {
            wheel_speeds[i] *= scale_factor;
        }
    }
    
    // Send commands to individual motors
    for (int i = 0; i < 4; i++) {
        DJI_Motor_Set_Velocity(chassis_motors[i], wheel_speeds[i]);
    }
}