#ifndef CONFIG_PINS_H
#define CONFIG_PINS_H

/*****************************************************************
* Ultiboard v2.0 pin assignment
******************************************************************/

#define MOTOR_0_STEP_PIN   A3
#define MOTOR_0_DIR_PIN    A1
#define MOTOR_0_STOP_PIN   A0
#define MOTOR_0_ENABLE_PIN A5

#define MOTOR_1_STEP_PIN   C5
#define MOTOR_1_DIR_PIN    C4
#define MOTOR_1_STOP_PIN   A4
#define MOTOR_1_ENABLE_PIN C6

#define MOTOR_2_STEP_PIN   C2
#define MOTOR_2_DIR_PIN    C1
#define MOTOR_2_STOP_PIN   A7
#define MOTOR_2_ENABLE_PIN C3

#define MOTOR_3_STEP_PIN   L7
#define MOTOR_3_DIR_PIN    L6
#define MOTOR_3_ENABLE_PIN C0

#define MOTOR_4_STEP_PIN   L0
#define MOTOR_4_DIR_PIN    L2
#define MOTOR_4_ENABLE_PIN L1

#define POWER_ON_PIN       A2

#define MOTOR_CURRENT_PWM_XY_PIN L5
#define MOTOR_CURRENT_PWM_Z_PIN  L4
#define MOTOR_CURRENT_PWM_E_PIN  L3
//Motor current PWM conversion, PWM value = MotorCurrentSetting * 255 / range
#define MOTOR_CURRENT_PWM_RANGE  2000L

#define LCD_PINS_D4     J1
#define LCD_PINS_ENABLE J0
#define LCD_PINS_D7     H3

#define ROTARY_ENCODER_PIN1 G1
#define ROTARY_ENCODER_PIN2 G0
#define ROTARY_BUTTON_PIN   D2

#define BEEPER  D3

#define SDCARDDETECT G2

#endif
