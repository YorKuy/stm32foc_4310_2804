/************************************************
本文件主要来自CSDN博主loop222对SimpleFOC项目的移植
原教程：https://blog.csdn.net/loop222/article/details/120471390
本项目删除了部分代码，添加了Flash数据读取和对蜂鸣功能的兼容
************************************************/

#include "BLDCMotor.h"
#include "FOCMotor.h"
#include "foc_utils.h"
#include "gpio.h"
#include <stdio.h>
#include "MagneticSensor.h" 
#include "tim.h"
#include "FlashStorage.h"

#define M1_Enable HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
#define M1_Disable HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define PWM_Period 1280

/* ===== 相位超前补偿（换相用）=====
 * 从 AS5600 读出角度，到 CCR 真正生效，中间有约 0.3~0.5 ms 延迟
 * （软件 I2C 读角 + 主循环 + PWM 更新）。高速时这点延迟会变成电角度滞后：
 * 300rpm / 14 对极时约 10 度，转矩按 cos(delta) 下降、脉动变大。
 * 这里按机械转速把电角度往前推 omega_elec * T_COMP 做补偿（转速为 0 时补偿为 0，不影响标定）。 */
#define PHASE_ADV_ENABLE 1
#define PHASE_ADV_TC     0.0004f   /* 等效延迟(s)，0.3~0.5ms 之间微调 */

extern float target;
extern float speed;   /* 机械转速(rpm)，定义在 main.c，供相位超前补偿使用 */

long sensor_direction;
float voltage_power_supply;
float voltage_limit;
float voltage_sensor_align;
int  pole_pairs;
unsigned long open_loop_timestamp;
float velocity_limit;
/* 调试用：把 setPhaseVoltage 算出的三相占空比暴露到全局，方便在 main 里打印 */
float Ta_dbg, Tb_dbg, Tc_dbg;
/* PWM 同步更新缓冲：setPhaseVoltage 只计算并存入，由 TIM2 更新中断在载波零点统一写 CCR。
 * 主循环里直接写 CCR 会让更新时刻落在载波周期中途，同一周期的三相脉冲宽度对应不同时刻的
 * 电角度 → 脉冲不对称 → 高速时表现为转矩脉动（用户说的"换相感"）。 */
volatile float pwmCcrA, pwmCcrB, pwmCcrC;   /* 已换算成比较值(0~PWM_Period)的三相占空比 */
volatile uint8_t pwmPending;                /* 1 = 有新的比较值等待装载 */

int alignSensor(void);
float velocityOpenloop(float target_velocity);
float angleOpenloop(float target_angle);
/******************************************************************************/
void Motor_init(void)
{
	printf("MOT: Init\r\n");
	if(voltage_sensor_align > voltage_limit)
		voltage_sensor_align = voltage_limit;
	
	pole_pairs=14;  // 4310 电机 24N28P = 14 对极（标定时会被实测值覆盖）
	sensor_direction=UNKNOWN;
	M1_Enable;
	//M1_Disable;
	printf("MOT: Enable driver.\r\n");
}
/******************************************************************************/
void Motor_initFOC(void)
{
	int flashOK = (Flash_ReadMotorParam(&pole_pairs, &zero_electric_angle, (int*)&sensor_direction) != -1);
	
	/* Flash 里的参数也可能是"坏标定"写进去的（例如标定时转子被机构限位挡住，
	 * 只转了 3.9 度，极对数被算成 93）。这种值一旦上电就用，电角度会被放大 6 倍，
	 * 电机只会震动不转。所以读出来后先做一次合理性检查，不合理就当作没读到、重新标定。 */
	if(flashOK && (pole_pairs < 1 || pole_pairs > 60 || (sensor_direction != 1 && sensor_direction != -1)))
	{
		printf("MOT: Flash param invalid (PP=%d dir=%d), force re-align\r\n", pole_pairs, (int)sensor_direction);
		flashOK = 0;
	}
	
	if(!flashOK) //尝试读取Flash数据
	{
		printf("MOT: no valid Flash param, start align...\r\n");
		if(alignSensor()) //读取识别则进行电机校准，检测零点偏移量和极对数
		{
			Flash_SaveMotorParam(pole_pairs, zero_electric_angle, sensor_direction); //校准数据存入Flash
			printf("MOT: align done, saved to Flash. PP=%d dir=%d\r\n", pole_pairs, (int)sensor_direction);
		}
	}
	else
		printf("MOT: Flash param loaded: PP=%d dir=%d\r\n", pole_pairs, (int)sensor_direction);
	
	//added the shaft_angle update
	angle_prev=getAngle();  //getVelocity(),make sure velocity=0 after power on
	HAL_Delay(5);
	shaft_angle = shaftAngle();// shaft angle
	
	HAL_Delay(200);
}
/******************************************************************************/
/*
 * alignSensor() —— 电机参数自动标定
 *
 * 【为什么要标定】
 * FOC 换相必须知道三件事，而这三件事在同一型号电机上也不完全一致，且与编码器安装角度有关：
 *   1) pole_pairs          —— 极对数（决定 电角度 = 机械角 × 极对数）
 *   2) sensor_direction    —— 编码器读数增大方向与实际磁场旋转方向是否同向（CW / CCW）
 *   3) zero_electric_angle —— 电角度零点偏移（d 轴与编码器零位之间的相位差）
 * 三者只要有一个不对，电机就会抖动、堵转或者反转，所以必须在闭环之前测出来。
 *
 * 【怎么测】
 * 给三相绕组施加一个幅值固定（voltage_sensor_align）、电角度缓慢线性旋转的电压矢量，
 * 转子会被这个旋转磁场"牵着"同步转一整圈电周期。转过的电角度是我们自己发出去的，
 * 转过的机械角由 AS5600 读出来，于是：
 *     极对数       = 2*PI / 机械角位移
 *     转向符号     = 机械角位移的符号
 *     零点偏移     = 最终"发出去的电角度"与"编码器电角度"之差
 *
 * 【调用时机】
 * 只在 Motor_initFOC() 里 Flash 参数无效（首次上电或被擦除）时调用一次；
 * 成功后由调用者把结果写入 Flash 第 30 页，之后上电直接读 Flash，不再标定。
 *
 * 【副作用】
 * 标定过程中电机会来回摆动约 2~3 秒，属正常现象；此时三相有电流，不要堵转。
 *
 * 返回：1 = 标定成功（三项结果已写入全局变量）；0 = 转子几乎没动，标定失败。
 */
/* newlib-nano 的浮点 printf 需要 -u _printf_float，会多占约 8.6KB Flash，
 * 32KB 的 F103C6 装不下；这里用整数运算打印 x.xxxx 格式，效果等价 */
static void PrintFloat4(const char *tag, float v)
{
	int32_t scaled = (int32_t)(v * 10000.0f + (v >= 0 ? 0.5f : -0.5f));
	int32_t ip = scaled / 10000;
	int32_t fp = scaled % 10000;
	if(fp < 0) fp = -fp;
	printf("%s%d.%04ld\r\n", tag, (int)ip, (long)fp);
}
//角度测量
int alignSensor(void)
{
	/* 探针量：正转一圈后的机械角、反转回来后的机械角、以及两者之差 */
	long i;
	float angle;
	float mid_angle,end_angle;
	float moved;
	
	printf("MOT: Align sensor.\r\n");
	
	/* ---------------- 第一步：判断编码器方向 ----------------
	 * 策略是"正转一圈再反转一圈"：
	 *   只转一次只能得到机械角位移的大小，无法区分"编码器读数增大"对应 CW 还是 CCW；
	 *   往返之后用两个终点的差值才能定出符号。
	 */
	// find natural direction
	// move one electrical revolution forward
	/* 正转：i 从 0 到 500 共 501 个点，每点停 2ms，约 1 秒转完一整圈电周期。
	 * 起点取 3*PI/2 而不是 0，是为了避开电角度 0（对齐点）附近的死区，
	 * 让正转和反转的起步条件对称。 */
	for(i=0; i<=500; i++)
	{
		//起点为3pi/2为了避开零点
		/* 第 i 步的目标电角度：3*PI/2 + 2*PI*(i/500)，即线性插值走完 2*PI */
		angle = _3PI_2 + _2PI * i / 500.0;
		/* 参数依次为：电压幅值、q 轴分量(电压控扭矩时不用)、电角度 */
		setPhaseVoltage(voltage_sensor_align, 0,  angle);
		/* 每步等 2ms 让转子跟上来；延时太短会丢步，算出的极对数就是错的 */
		HAL_Delay(2);
	}
	/* 正转终点的编码器读数（弧度，已归一化到 0~2*PI） */
	mid_angle=getAngle();
	
	/* 反转：电角度沿同样轨迹倒着走回去，带动转子转回起点附近 */
	for(i=500; i>=0; i--) 
	{
		angle = _3PI_2 + _2PI * i / 500.0 ;
		setPhaseVoltage(voltage_sensor_align, 0,  angle);
		HAL_Delay(2);
	}
	/* 反转终点的读数；正常情况下与 mid_angle 相差约 2*PI/pole_pairs（即一整圈电周期对应的机械角） */
	end_angle=getAngle();
	/* 三相占空比全部归零，撤掉电压矢量，避免线圈持续发热 */
	setPhaseVoltage(0, 0, 0);
	/* 等转子彻底停稳；还在转时读数会漂，方向判断就会错 */
	HAL_Delay(200);
	
	PrintFloat4("mid_angle=", mid_angle);
	PrintFloat4("end_angle=", end_angle);
	
	/* moved = 往返一圈电周期对应的总机械角位移（取绝对值） */
	moved =  fabs(mid_angle - end_angle);
	/* 判定"电机没动"：两次读数完全相等，或机械角位移小于 0.02 rad（约 1.1°）。
	 * 常见原因：没接电机、三相线断相、驱动没使能、编码器无信号、对齐电压太小。 */
	if((mid_angle == end_angle)||(moved < 0.02))  //相等或者几乎没有动
	{
		printf("MOT: Failed to notice movement loop222.\r\n");
		/* 标定失败就关掉驱动，避免继续通电发热；不写 Flash，下次上电会重试 */
		M1_Disable;    //电机检测不正常，关闭驱动
		return 0;
	}
	/* 方向判定：mid_angle < end_angle 说明"磁场正转"时编码器读数在减小，
	 * 即编码器计数方向与相序相反，记为 CCW；反之记为 CW。
	 * 后续所有电角度换算都会乘上 sensor_direction 做符号修正。 */
	else if(mid_angle < end_angle)
	{
		printf("MOT: sensor_direction==CCW\r\n");
		sensor_direction=CCW;
	}
	else
	{
		printf("MOT: sensor_direction==CW\r\n");
		sensor_direction=CW;
	}
	
	
	/* ---------------- 第二步：校验极对数 ----------------
	 * 电角度转过 2*PI 时，机械角应正好转过 2*PI / pole_pairs（本工程默认 7）。
	 * 若实测位移与假设的极对数对不上（偏差超过 0.5 rad），就用实测值反算极对数。 */
	printf("MOT: PP check: ");    //计算Pole_Pairs
	/* 0.5 rad 是经验容差：太小会把传感器抖动当成错误，太大则漏判极对数错误 */
	if( fabs(moved*pole_pairs - _2PI) > 0.5 )  // 0.5 is arbitrary number it can be lower or higher!
	{
		printf("fail - estimated pp:");
		/* 反算极对数：2*PI / 实测机械角位移，+0.5 实现四舍五入后转成整型 */
		pole_pairs=_2PI/moved+0.5;     //浮点数转整形，四舍五入
		printf("%d\r\n",pole_pairs);
  }
	else
		printf("OK!\r\n");
	
	
	/* ---------------- 第三步：测量电角度零点 ----------------
	 * 把电压矢量固定停在电角度 3*PI/2，让转子被磁场牢牢吸在这个位置不动。
	 * 700ms 是等转子完全稳定的时间：有负载或惯量大时收敛慢，时间不够会带来固定偏差。 */
	setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2);  //计算零点偏移角度
	HAL_Delay(700);
	/* 此时"发出去的电角度"是 3*PI/2，转子被磁场吸在该电角度对应的位置；
	 * 读回来的编码器机械角经 sensor_direction 修正后乘以极对数，就是"编码器电角度"，
	 * 记入 zero_electric_angle。之后 electricalAngle() 的换相角计算会把它减掉：
	 *     electrical_angle = _normalizeAngle((shaft_angle + sensor_offset)*pole_pairs - zero_electric_angle)
	 * 这个减法就是零点补偿，保证电压矢量与转子的 d 轴对齐。
	 * 注意 _electricalAngle() 只做"机械角×极对数"，取模与归一化由 _normalizeAngle() 完成。 */
	zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction*getAngle(), pole_pairs));
	/* 留一点时间让编码器采样与滤波稳定后再读 */
	HAL_Delay(20);
	printf("MOT: Zero elec. angle:");
	PrintFloat4("", zero_electric_angle);
	
	/* 标定结束，撤掉电压矢量，电机不再发热 */
	setPhaseVoltage(0, 0, 0);
	/* 等转子停稳，保证返回后 Motor_initFOC() 读到的 shaft_angle 是静止角 */
	HAL_Delay(200);
	
	/* 合理性校验：极对数必须落在 1~60 之间。
	 * 如果标定时转子几乎没动（负载/限位/缺相），对齐只转了 3~4 度，
	 * 极对数会按 2*PI/moved 算出 90 多这种荒谬值并写进 Flash，
	 * 之后每次上电都拿它算电角度，电机就只会震动不转。这里拦住它。 */
	if(pole_pairs < 1 || pole_pairs > 60 || fabs(moved*pole_pairs - _2PI) > 0.5)
	{
		printf("MOT: Align failed - bad pole pairs %d, NOT saved.\r\n", pole_pairs);
		setPhaseVoltage(0, 0, 0);
		M1_Disable;
		return 0;
	}
	
	/* 返回 1：调用者据此把 pole_pairs / zero_electric_angle / sensor_direction 写入 Flash */
	return 1;
}
/******************************************************************************/
void loopFOC(void)
{
	if( controller==Type_angle_openloop || controller==Type_velocity_openloop ) return;  //如果是角度开环和电压开环
	
	shaft_angle = shaftAngle();// shaft angle
	electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first

#if PHASE_ADV_ENABLE
	/* 相位超前补偿：电角度 = 电角度 + omega_elec * T_COMP，抵消"读角→PWM 生效"的延迟。
	 * omega_elec(rad/s) = speed(rpm) * 2pi/60 * pole_pairs */
	if(speed != 0.0f)
		electrical_angle = _normalizeAngle(electrical_angle + speed * (2.0f*_PI/60.0f) * pole_pairs * PHASE_ADV_TC);
#endif
	
	switch(torque_controller)
	{
		case Type_voltage:  // no need to do anything really
			break;
		case Type_dc_current:
			break;
		case Type_foc_current:
			break;
		default:
			printf("MOT: no torque control selected!");
			break;
	}
	// set the phase voltage - FOC heart function :)
	extern uint8_t beepPlaying;
	if(beepPlaying == 0) //没在蜂鸣状态则正常输出电压
		setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}
/******************************************************************************/
void setTargetVotage(float new_target)
{
	voltage.q = new_target;
}
/******************************************************************************/
/* setPhaseVoltage()：FOC 的核心，把 dq 轴电压指令变成三相半桥的 PWM 占空比。
 *
 * 调制方式是 SVPWM（空间矢量调制）。三相半桥只有 8 种开关组合：
 *   - 6 个有效矢量，方向相差 60 度，幅值 2/3*Vbus；
 *   - 2 个零矢量（三相全接电源 / 全接地），输出电压为 0。
 * 想要的方向一定落在某个 60 度扇区内，于是就在一个 PWM 周期里让"扇区两侧的两个相邻矢量"
 * 和"零矢量"分时输出，按伏秒积分等效地把目标矢量合成出来。
 */
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	/* Uout：归一化后的矢量幅值；
	 * T1/T2：扇区两侧两个基本矢量各自的作用时间，以 PWM 周期为 1；
	 * T0：剩下的零矢量时间；Ta/Tb/Tc：最终三相占空比（0~1）；sector：扇区号 1~6。 */
	float Uout;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;
	
	if(Ud) // only if Ud and Uq set 
	{// _sqrt is an approx of sqrt (3-4% error)
		/* 幅值归一化：矢量长度是 sqrt(Ud^2+Uq^2)，除以母线电压后 Uout=1 表示
		 * 矢量幅值等于母线电压本身（实际调制上限只到 1/sqrt(3)，见下面钳位）；
		 * _sqrt() 是快速开方近似（误差 3~4%），省算力，对 25kHz 的实时性有意义 */
		Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		/* 矢量方向 = 电角度 + atan2(Uq,Ud)。归一化到 [0,2PI) 是硬性要求：
		 * 下面的 _sin() 是查表插值，只在这个区间内有效 */
		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
	}
	else
	{// only Uq available - no need for atan2 and sqrt
		/* 只有 q 轴分量：幅值就是 Uq/母线电压。Uq 可以为负，负值相当于把矢量反向，
		 * 也就是输出反向力矩，这里不取绝对值，负号由后面的 T1/T2 变负自然体现。
		 * 方向固定比转子电角度超前 90 度：q 轴本来就垂直于 d 轴 */
		Uout = Uq / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		/* +PI/2 把"转子 d 轴方向"换算成"电压矢量方向" */
		angle_el = _normalizeAngle(angle_el + _PI_2);
	}
	if(Uout> 0.577)Uout= 0.577;
	if(Uout<-0.577)Uout=-0.577;
	
	/* 扇区号：每个扇区 60 度（PI/3），(angle/PI/3) 向下取整后 +1 得 1~6。
	 * 因为前面已把 angle_el 归一化到 [0,2PI)，不会出现 0 或 7；
	 * 赋给 uint32_t 时自动截断小数，正好完成向下取整 */
	sector = (angle_el / _PI_3) + 1;
	/* 用正弦定理（伏秒平衡）解出两个相邻基本矢量各占多长时间：
	 *   T1 = sqrt(3)*sin(扇区起始边界+60度 - 矢量角)*Uout —— 扇区起始边界（角度较小）的基本矢量
	 *   T2 = sqrt(3)*sin(矢量角 - 扇区起始边界)*Uout       —— 扇区结束边界（角度较大）的基本矢量
	 * 系数 sqrt(3) 来自基本矢量幅值为 2/3*Vbus、而我们用相电压峰值做了归一化。
	 * 目标矢量正好与某个基本矢量同向时，对应的 T 为 0；指向扇区正中时 T1=T2。
	 * 两者非负且和恒小于 1（因为 Uout 已钳位在 1/sqrt(3)）。
	 * T0 是补齐一个 PWM 周期的剩余时间，给零矢量（三相同时接地或同时接电源，合成电压为零） */
	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
	T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;
	T0 = 1 - T1 - T2;
	
	// calculate the duty cycles(times)
	/* 把 (T1,T2,T0) 翻译成三相占空比。每个扇区里总有一相只由 T0/2 构成，
	 * 另外两相由 T1 与 T2 的组合构成，具体哪一相是 T1、哪一相是 T2 由扇区决定，
	 * 下面六个 case 就是六边形上六个扇区的查表。
	 * 用 T0/2 而不是 T0：零矢量被平分到 PWM 周期的首尾两端，形成中间对齐（中心对称）调制，
	 * 三相波形以周期中点对称，共模谐波更小、电流纹波更均匀 */
	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	
	Ta_dbg = Ta; Tb_dbg = Tb; Tc_dbg = Tc; // 调试：抄一份给 main 打印

	/* 不在这里直接写 CCR：存进缓冲并置标志，由 TIM2 更新中断（载波零点）统一装载 */
	pwmCcrA = Ta*PWM_Period;
	pwmCcrB = Tb*PWM_Period;
	pwmCcrC = Tc*PWM_Period;
	pwmPending = 1;
}
