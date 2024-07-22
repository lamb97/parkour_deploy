#include "motor_control.h"

// #define KP_MIN 0.0f
// #define KP_MAX 500.0f
// #define KD_MIN 0.0f
// #define KD_MAX 5.0f
// #define POS_MIN -12.5f
// #define POS_MAX 12.5f
// #define SPD_MIN -25.0f
// #define SPD_MAX 25.0f
// #define T_MIN -50.0f
// #define T_MAX 50.0f
// #define I_MIN -40.0f
// #define I_MAX 40.0f// OLD

#define POS_MIN -12.5
#define POS_MAX 12.5
#define SPD_MIN -65
#define SPD_MAX 65
#define KP_MIN 0
#define KP_MAX 500
#define KD_MIN 0
#define KD_MAX 50
#define T_MIN -150
#define T_MAX 150 // L T

// kd=[0,50],T=[-30,30],I=[-30,30]
union RV_TypeConvert
{
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
} rv_type_convert;

union RV_TypeConvert2
{
    int16_t to_int16;
    uint16_t to_uint16;
    uint8_t buf[2];
} rv_type_convert2;

MotorCommFbd motor_comm_fbd;
OD_Motor_Msg rv_motor_msg[12];
IMU_Msg imu_msg;

void send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage, uint16_t motor_id, float kp, float kd, float pos, float spd, float tor)
{
    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;
    int i = 0;
    if (motor_id == 6)
    {
        TxMessage->motor[5].id = 3;
        i = 5;
    }
    if (motor_id == 5)
    {
        TxMessage->motor[4].id = 2;
        i = 4;
    }
    if (motor_id == 4)
    {
        TxMessage->motor[3].id = 1;
        i = 3;
    }

    if (motor_id == 3)
    {
        TxMessage->motor[2].id = 3;
        i = 2;
    }
    if (motor_id == 2)
    {
        TxMessage->motor[1].id = 2;
        i = 1;
    }
    if (motor_id == 1)
    {
        TxMessage->motor[0].id = 1;
        i = 0;
    }
    TxMessage->can_ide = 0;
    TxMessage->motor[i].rtr = 0;
    TxMessage->motor[i].dlc = 8;
    if (kp > KP_MAX)
        kp = KP_MAX;
    else if (kp < KP_MIN)
        kp = KP_MIN;
    if (kd > KD_MAX)
        kd = KD_MAX;
    else if (kd < KD_MIN)
        kd = KD_MIN;
    if (pos > POS_MAX)
        pos = POS_MAX;
    else if (pos < POS_MIN)
        pos = POS_MIN;
    if (spd > SPD_MAX)
        spd = SPD_MAX;
    else if (spd < SPD_MIN)
        spd = SPD_MIN;
    if (tor > T_MAX)
        tor = T_MAX;
    else if (tor < T_MIN)
        tor = T_MIN;
    /*new*/
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    tor_int = float_to_uint(tor, T_MIN, T_MAX, 12);
    // printf("send_row_data:p_v_kp_kd_t%f %f %f %f %f--end\n",pos_int, spd_int, kp_int, kd_int,tor_int);

    TxMessage->motor[i].data[0] = pos_int >> 8;   // kp5
    TxMessage->motor[i].data[1] = pos_int & 0xFF; // kp7+kd1
    TxMessage->motor[i].data[2] = spd_int >> 4;
    TxMessage->motor[i].data[3] = ((spd_int & 0xF) << 4) | (kp_int >> 8);
    TxMessage->motor[i].data[4] = kp_int & 0xFF;
    TxMessage->motor[i].data[5] = kd_int >> 4;
    TxMessage->motor[i].data[6] = ((kd_int & 0xF) << 4) | (tor_int >> 8);
    TxMessage->motor[i].data[7] = tor_int & 0xff;
}

void RV_can_data_repack(EtherCAT_Msg *RxMessage, uint8_t comm_mode, uint8_t slave_id)
{
    // printf("slave %d msg:\n", slave_id);
    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;

    // Tao code

    for (int i = 0; i <= 5; i++)
    {
        if (RxMessage->motor[i].dlc == 0)
        {
            continue;
        }
        // // printf("\n   i=%d RxMessage->motor->id %d,DATA is %d-%d-%d-%d-%d-%d-%d-%d\n", i, RxMessage->motor[i].id, RxMessage->motor[i].data[0], RxMessage->motor[i].data[1], RxMessage->motor[i].data[2], RxMessage->motor[i].data[3], RxMessage->motor[i].data[4], RxMessage->motor[i].data[5], RxMessage->motor[i].data[6], RxMessage->motor[i].data[7]);

        ack_status = RxMessage->motor[i].id - 1;
        //printf("motor[%d].data[0]:%u,RxMessage->motor[].data[1]: %u\n ",i ,RxMessage->motor[i].data[0], RxMessage->motor[i].data[1]);
        if (slave_id == 0)
        {
            if (i < 3)
            {
                motor_id_t = RxMessage->motor[i].data[0];
            }
            else
            {
                motor_id_t = RxMessage->motor[i].data[0] + 3;
            }
        }
        else
        {
            if (i < 3)
            {
                motor_id_t = RxMessage->motor[i].data[0] + 6;
            }
            else
            {
                motor_id_t = RxMessage->motor[i].data[0] + 9;
            }
        }
        // Tao code end
        pos_int = (RxMessage->motor[i].data[1] << 8) | (0x00ff & RxMessage->motor[i].data[2]);
        pos_int &= 0x0000ffff;
        spd_int = (RxMessage->motor[i].data[3] << 4) | ((0x00ff & RxMessage->motor[i].data[4]) >> 4);
        spd_int &= 0x00000fff;
        cur_int = ((RxMessage->motor[i].data[4] & 0xF) << 8) | (0x00ff & RxMessage->motor[i].data[5]);
        cur_int &= 0x00000fff;

        rv_motor_msg[motor_id_t].angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
        rv_motor_msg[motor_id_t].speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
        rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, T_MIN, T_MAX, 12);
    }
}

void RV_can_imu_data_repack(EtherCAT_Msg *RxMessage)
{
    uint16_t angle_uint[3], gyro_uint[3], accel_uint[3], mag_uint[3];
    // uint32_t quat_uint[4];
    // float angle_float[3],gyro_float[3],accel_float[3],mag_float[3],quat_float[4];

    rv_type_convert2.buf[0] = RxMessage->motor[0].data[1];
    rv_type_convert2.buf[1] = RxMessage->motor[0].data[0];
    angle_uint[0] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[0].data[3];
    rv_type_convert2.buf[1] = RxMessage->motor[0].data[2];
    angle_uint[1] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[0].data[5];
    rv_type_convert2.buf[1] = RxMessage->motor[0].data[4];
    angle_uint[2] = rv_type_convert2.to_uint16;

    rv_type_convert2.buf[0] = RxMessage->motor[0].data[7];
    rv_type_convert2.buf[1] = RxMessage->motor[0].data[6];
    gyro_uint[0] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[1].data[1];
    rv_type_convert2.buf[1] = RxMessage->motor[1].data[0];
    gyro_uint[1] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[1].data[3];
    rv_type_convert2.buf[1] = RxMessage->motor[1].data[2];
    gyro_uint[2] = rv_type_convert2.to_uint16;

    rv_type_convert2.buf[0] = RxMessage->motor[1].data[5];
    rv_type_convert2.buf[1] = RxMessage->motor[1].data[4];
    accel_uint[0] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[1].data[7];
    rv_type_convert2.buf[1] = RxMessage->motor[1].data[6];
    accel_uint[1] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[2].data[1];
    rv_type_convert2.buf[1] = RxMessage->motor[2].data[0];
    accel_uint[2] = rv_type_convert2.to_uint16;

    rv_type_convert2.buf[0] = RxMessage->motor[2].data[3];
    rv_type_convert2.buf[1] = RxMessage->motor[2].data[2];
    mag_uint[0] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[2].data[5];
    rv_type_convert2.buf[1] = RxMessage->motor[2].data[4];
    mag_uint[1] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[2].data[7];
    rv_type_convert2.buf[1] = RxMessage->motor[2].data[6];
    mag_uint[2] = rv_type_convert2.to_uint16;

    rv_type_convert.buf[0] = RxMessage->motor[3].data[3];
    rv_type_convert.buf[1] = RxMessage->motor[3].data[2];
    rv_type_convert.buf[2] = RxMessage->motor[3].data[1];
    rv_type_convert.buf[3] = RxMessage->motor[3].data[0];
    // quat_uint[0] = rv_type_convert.to_uint;
    imu_msg.quat_float[0] = rv_type_convert.to_float;
    rv_type_convert.buf[0] = RxMessage->motor[3].data[7];
    rv_type_convert.buf[1] = RxMessage->motor[3].data[6];
    rv_type_convert.buf[2] = RxMessage->motor[3].data[5];
    rv_type_convert.buf[3] = RxMessage->motor[3].data[4];
    // quat_uint[1] = rv_type_convert.to_uint;
    imu_msg.quat_float[1] = rv_type_convert.to_float;
    rv_type_convert.buf[0] = RxMessage->motor[4].data[3];
    rv_type_convert.buf[1] = RxMessage->motor[4].data[2];
    rv_type_convert.buf[2] = RxMessage->motor[4].data[1];
    rv_type_convert.buf[3] = RxMessage->motor[4].data[0];
    // quat_uint[2] = rv_type_convert.to_uint;
    imu_msg.quat_float[2] = rv_type_convert.to_float;
    rv_type_convert.buf[0] = RxMessage->motor[4].data[7];
    rv_type_convert.buf[1] = RxMessage->motor[4].data[6];
    rv_type_convert.buf[2] = RxMessage->motor[4].data[5];
    rv_type_convert.buf[3] = RxMessage->motor[4].data[4];
    // quat_uint[3] = rv_type_convert.to_uint;
    imu_msg.quat_float[3] = rv_type_convert.to_float;

    // uint to float
    imu_msg.angle_float[0] = uint_to_float(angle_uint[0], -3.5, 3.5, 16);
    imu_msg.angle_float[1] = uint_to_float(angle_uint[1], -3.5, 3.5, 16);
    imu_msg.angle_float[2] = uint_to_float(angle_uint[2], -3.5, 3.5, 16);

    imu_msg.gyro_float[0] = uint_to_float(gyro_uint[0], -35.0, 35.0, 16);
    imu_msg.gyro_float[1] = uint_to_float(gyro_uint[1], -35.0, 35.0, 16);
    imu_msg.gyro_float[2] = uint_to_float(gyro_uint[2], -35.0, 35.0, 16);

    imu_msg.accel_float[0] = uint_to_float(accel_uint[0], -240.0, 240.0, 16);
    imu_msg.accel_float[1] = uint_to_float(accel_uint[1], -240.0, 240.0, 16);
    imu_msg.accel_float[2] = uint_to_float(accel_uint[2], -240.0, 240.0, 16);

    imu_msg.mag_float[0] = uint_to_float(mag_uint[0], -250.0, 250.0, 16);
    imu_msg.mag_float[1] = uint_to_float(mag_uint[1], -250.0, 250.0, 16);
    imu_msg.mag_float[2] = uint_to_float(mag_uint[2], -250.0, 250.0, 16);
}