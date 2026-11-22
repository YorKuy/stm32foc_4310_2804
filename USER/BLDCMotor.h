#ifndef BLDCMotor_H
#define BLDCMotor_H

#include <stdint.h>

/******************************************************************************/
/**
 *  Direction structure
 */
typedef enum
{
    CW      = 1,  //clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   //not yet known or invalid state
} Direction;

/******************************************************************************/
extern long sensor_direction;
extern float voltage_power_supply;
extern float Ta_dbg, Tb_dbg, Tc_dbg;
/* PWM 同步更新缓冲（定义在 BLDCMotor.c）：setPhaseVoltage 只存值，TIM2 更新中断负责装载 */
extern volatile float pwmCcrA, pwmCcrB, pwmCcrC;
extern volatile uint8_t pwmPending;
extern float voltage_limit;
extern float voltage_sensor_align;
extern int  pole_pairs;
extern unsigned long open_loop_timestamp;
extern float velocity_limit;
/******************************************************************************/
void Motor_init(void);
void Motor_initFOC(void);
void loopFOC(void);
void setTargetVotage(float new_target);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
/******************************************************************************/

#endif
