//========================================================================
//  自动飞行控制增强版 control.c（包含完整函数）
//========================================================================
#include "ALL_DATA.h" 
#include "ALL_DEFINE.h" 
#include "control.h"
#include "pid.h"
#include "flow.h"
#include "kalman.h"

#undef NULL
#define NULL 0
#undef DISABLE 
#define DISABLE 0
#undef ENABLE 
#define ENABLE 1
#undef REST
#define REST 0
#undef SET 
#define SET 1
#undef EMERGENT
#define EMERGENT 0

PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw,
                           &pidHeightRate,&pidHeightHigh,&pidPosRateX,&pidPosRateY,
                           &pidPositionX,&pidPositionY};

float sins_high;
float sins_vel;
uint16_t thr_hold = 0;

uint32_t VL53L01_high = 0;
static uint8_t high_error_count;
uint8_t set_high = 0;

// 自动飞行状态机定义
typedef enum {
    AUTO_WAIT = 0,
    AUTO_TAKEOFF,
    AUTO_HOVER,
		AUTO_FORWARD,
	  AUTO_LEFTWARD,
    AUTO_LAND,
    AUTO_FINISH
} AutoFlightState;

AutoFlightState auto_flight_state = AUTO_WAIT;
uint16_t auto_counter = 0;

void AutoFlightManager(void)//自动控制代码 在scheduler.c里面调用，每3ms一个循环,因此，时间是循环数乘上3ms
{
    switch (auto_flight_state)
    {
        case AUTO_WAIT://上电后等待状态
            auto_counter++;
				    Remote.pitch=1700;
				    Remote.roll=1479;
				    Remote.yaw=1509;
            if (auto_counter > 5000)//上电后等待15秒
            {
								Angle.pitch=0;
							  Angle.roll=0;
							  Angle.yaw=0;
                auto_counter = 0;
                auto_flight_state = AUTO_TAKEOFF;
                ALL_flag.unlock = 1;//解锁标志为1
                LED.status = AlwaysOn;
            }
            break;
        case AUTO_TAKEOFF://Task 1 起飞
            Remote.thr = 1400;//油门设置为1400，可以更高
            auto_counter++;
						if (Remote.pitch < 1800)
						{
						      Remote.pitch+=1.5;
						}
            if (auto_counter > 500)//三秒
            {
                auto_counter = 0;
                auto_flight_state = AUTO_HOVER;//进入悬停状态机
                ALL_flag.height_lock = 1;//进入悬停模式 
					      ALL_flag.flow_control =1;//进入光流定点模式
            }
            break;
        case AUTO_HOVER:
            Remote.thr = 1350;//定高油门，悬停定高状态下油门决定高度>1750上升
            auto_counter++;
            if (auto_counter > 1000)//悬停3秒
            {
                auto_counter = 0;
                auto_flight_state = AUTO_FORWARD;//进入降落模式
            }
            break;
				case AUTO_FORWARD:// Task2
					   
						 Remote.pitch = 1800;
				     Angle.yaw -= 0.008;
						 auto_counter++;
				     if (auto_counter > 4500)
            {
                auto_counter = 0;
                auto_flight_state = AUTO_LEFTWARD;//进入Task3
            }
				
						break;
					case AUTO_LEFTWARD:// Task3
					   
						 Remote.pitch = 1531;
					   Remote.roll=1200;
					   Angle.yaw -= 0.008;
						 auto_counter++;
				     if (auto_counter > 4500)
            {
                auto_counter = 0;
                auto_flight_state = AUTO_LAND;//进入降落模式
            }
				
						break;
					
        case AUTO_LAND://降落模式
            Remote.thr = 1100;
            auto_counter++;
            if (auto_counter > 100)
            {
                auto_flight_state = AUTO_FINISH;
                ALL_flag.height_lock = 0;
							  ALL_flag.flow_control=0;
            }
            break;
        case AUTO_FINISH:
            Remote.thr = 1000;
            ALL_flag.unlock = 0;//上锁
            LED.status = AllFlashLight;
            break;
    }
}
void FlowPidControl(float dt)
{
    static uint8_t status = WAITING_1;
    static uint8_t set_pos = 0;

    if (ALL_flag.unlock == EMERGENT)
        status = EXIT_255;

    switch (status)
    {
        case WAITING_1:
            if (ALL_flag.unlock)
                status = WAITING_2;
            break;
        case WAITING_2:
            if (ALL_flag.flow_control && VL53L01_high > 200)
            {
                pidRest(&pPidObject[8], 4);
                status = PROCESS_31;
                set_pos = 1;
            }
            break;
        case PROCESS_31:
            if (Remote.roll > 1750 || Remote.roll < 1250 || Remote.pitch > 1750 || Remote.pitch < 1250)
            {
                set_pos = 1;
                pidPosRateX.desired = (Remote.roll > 1750) ? -15 : (Remote.roll < 1250 ? 15 : 0);
                pidPosRateY.desired = (Remote.pitch > 1750) ? 15 : (Remote.pitch < 1250 ? -15 : 0);
            }
            else
            {
                if (set_pos)
                {
                    set_pos = 0;
                    pidPositionX.desired = flow_x_lpf_att_i;
                    pidPositionY.desired = flow_y_lpf_att_i;
                }
                pidPositionX.measured = flow_x_lpf_att_i;
                pidUpdate(&pidPositionX, dt);
                pidPositionY.measured = flow_y_lpf_att_i;
                pidUpdate(&pidPositionY, dt);
                pidPosRateX.desired = LIMIT(pidPositionX.out, -20, 20);
                pidPosRateY.desired = LIMIT(pidPositionY.out, -20, 20);
            }
            pidPosRateX.measured = flow_x_vel_lpf_i;
            pidUpdate(&pidPosRateX, dt);
            pidPosRateY.measured = flow_y_vel_lpf_i;
            pidUpdate(&pidPosRateY, dt);

            pidRoll.desired = LIMIT(pidPosRateX.out, -15, 15);
            pidPitch.desired = -LIMIT(pidPosRateY.out, -15, 15);

            if (!ALL_flag.flow_control || !ALL_flag.height_lock)
                status = EXIT_255;
            break;
        case EXIT_255:
            pidRest(&pPidObject[8], 4);
            status = WAITING_1;
            break;
        default:
            status = WAITING_1;
            break;
    }
}
void HeightPidControl(float dt)
{
    static uint8_t status = WAITING_1;
    int16_t acc = (int16_t)GetNormAccz();
    static int16_t acc_offset = 0;
    int16_t acc_error = acc - acc_offset;

    if (!ALL_flag.unlock)
        acc_offset = acc;

    if (VL53L01_high < 3500)
    {
        if (VL53L01_high - sins_high > 50) sins_high += 50;
        else if (VL53L01_high - sins_high < -50) sins_high -= 50;
        else sins_high = VL53L01_high;

        sins_vel = (sins_vel + acc_error * dt) * 0.985f + 0.015f * (sins_high - sins_high) / dt;

        pidHeightRate.measured = sins_vel;
        pidHeightHigh.measured = sins_high;
    }

    if (ALL_flag.unlock == EMERGENT)
        status = EXIT_255;

    switch (status)
    {
        case WAITING_1:
            if (ALL_flag.unlock)
            {
                pidHeightRate.measured = sins_vel = 0;
                pidHeightHigh.measured = sins_high;
                status = WAITING_2;
                high_error_count = 0;
            }
            break;
        case WAITING_2:
            if (ALL_flag.height_lock)
            {
                set_high = 0;
                LED.status = WARNING;
                thr_hold = 500;
                status = PROCESS_31;
            }
            break;
        case PROCESS_31:
            if (Remote.thr < 1750 && Remote.thr > 1150)
            {
                if (!set_high)
                {
                    set_high = 1;
                    pidHeightHigh.desired = pidHeightHigh.measured;
                }
                pidUpdate(&pidHeightHigh, dt);
                pidHeightRate.desired = pidHeightHigh.out;
            }
            else if (Remote.thr > 1750)
            {
                if (VL53L01_high < 3500)
                {
                    set_high = 0;
                    pidHeightRate.desired = 250;
                }
            }
            else if (Remote.thr < 1150)
            {
                set_high = 0;
                pidHeightRate.desired = -350;
                if (pidHeightHigh.measured < 10)
                    ALL_flag.unlock = 0;
            }

            pidUpdate(&pidHeightRate, dt);

            if (!ALL_flag.height_lock || VL53L01_high < 50 || VL53L01_high > 3700)
            {
                high_error_count++;
                if (high_error_count > 50)
                {
                    ALL_flag.height_lock = 0;
                    ALL_flag.flow_control = 0;
                    status = EXIT_255;
                }
            }
            else
            {
                high_error_count = 0;
            }
            break;
        case EXIT_255:
            pidRest(&pPidObject[6], 2);
            status = WAITING_1;
            break;
        default:
            status = WAITING_1;
            break;
    }
}
void FlightPidControl(float dt)
{
    static uint8_t status = WAITING_1;
    switch (status)
    {
        case WAITING_1:
            if (ALL_flag.unlock)
                status = READY_11;
            break;
        case READY_11:
            pidRest(pPidObject, 6);
            Angle.yaw = pidYaw.desired = pidYaw.measured = 0;
            status = PROCESS_31;
            break;
        case PROCESS_31:
            if ((Angle.pitch < -50 || Angle.pitch > 50 || Angle.roll < -50 || Angle.roll > 50) && Remote.thr > 1200)
                ALL_flag.unlock = EMERGENT;
            pidRateX.measured = MPU6050.gyroX * Gyro_G;
            pidRateY.measured = MPU6050.gyroY * Gyro_G;
            pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
            pidPitch.measured = Angle.pitch;
            pidRoll.measured = Angle.roll;
            pidYaw.measured = Angle.yaw;
            pidUpdate(&pidRoll, dt);
            pidRateX.desired = pidRoll.out;
            pidUpdate(&pidRateX, dt);
            pidUpdate(&pidPitch, dt);
            pidRateY.desired = pidPitch.out;
            pidUpdate(&pidRateY, dt);
            pidUpdate(&pidYaw, dt);
            pidRateZ.desired = pidYaw.out;
            pidUpdate(&pidRateZ, dt);
            break;
        case EXIT_255:
            pidRest(pPidObject, 6);
            status = WAITING_1;
            break;
        default:
            status = EXIT_255;
            break;
    }
    if (ALL_flag.unlock == EMERGENT)
        status = EXIT_255;
}

#define MOTOR1 motor_PWM_Value[0] 
#define MOTOR2 motor_PWM_Value[1] 
#define MOTOR3 motor_PWM_Value[2] 
#define MOTOR4 motor_PWM_Value[3] 

uint16_t low_thr_cnt;

void MotorControl(void)
{
    static uint8_t status = WAITING_1;
    if (ALL_flag.unlock == EMERGENT)
        status = EXIT_255;

    switch (status)
    {
        case WAITING_1:
            MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;
            if (ALL_flag.unlock)
                status = WAITING_2;
            break;
        case WAITING_2:
            if (Remote.thr > 1100)
                status = PROCESS_31;
            break;
        case PROCESS_31:
        {
            int16_t thr;
            if (ALL_flag.height_lock)
            {
                thr = pidHeightRate.out + thr_hold;
            }
            else
            {
                int16_t temp = Remote.thr - 1000;
                thr = 200 + 0.45f * temp;
                thr_hold = thr;
                if (temp < 10)
                {
                    low_thr_cnt++;
                    if (low_thr_cnt > 300)
                    {
                        thr = 0;
                        pidRest(pPidObject, 6);
                        MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;
                        status = WAITING_2;
                        break;
                    }
                }
                else
                {
                    low_thr_cnt = 0;
                }
            }
            MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(thr, 0, 800);
            MOTOR1 += pidRateX.out + pidRateY.out + pidRateZ.out;
            MOTOR2 += pidRateX.out - pidRateY.out - pidRateZ.out;
            MOTOR3 += -pidRateX.out + pidRateY.out - pidRateZ.out;
            MOTOR4 += -pidRateX.out - pidRateY.out + pidRateZ.out;
        }
        break;
        case EXIT_255:
            MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;
            status = WAITING_1;
            break;
        default:
            break;
    }

    TIM2->CCR1 = LIMIT(MOTOR1, 0, 1000);
    TIM2->CCR2 = LIMIT(MOTOR2, 0, 1000);
    TIM2->CCR3 = LIMIT(MOTOR3, 0, 1000);
    TIM2->CCR4 = LIMIT(MOTOR4, 0, 1000);
}