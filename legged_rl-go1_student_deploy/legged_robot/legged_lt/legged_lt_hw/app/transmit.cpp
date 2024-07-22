extern "C"
{
#include "ethercat.h"
#include "motor_control.h"
#include "transmit.h"
}
#include <iostream>
#include "queue.h"
#include <sys/time.h>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#define EC_TIMEOUTM

spsc_queue<EtherCAT_Msg_ptr, capacity<10>> messages[SLAVE_NUMBER];
std::atomic<bool> running{false};
std::thread runThread;
// struct YKSMotorData
// {
// double pos_, vel_, tau_;                   // state
// double pos_des_, vel_des_, kp_, kd_, ff_;  // command
// };
// struct YKSMotorData motorDate_recv[12];
YKSMotorData motorDate_recv[12];
YKSIMUData imuData_recv;
char IOmap[4096];
OSAL_THREAD_HANDLE checkThread;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
uint64_t num;
bool isConfig[SLAVE_NUMBER]{false};

#define EC_TIMEOUTMON 500

void EtherCAT_Data_Get();

void EtherCAT_Get_State();
void EtherCAT_Send_Command(YKSMotorData *data);

static void degraded_handler()
{
    printf("[EtherCAT Error] Logging error...\n");
    time_t current_time = time(NULL);
    char *time_str = ctime(&current_time);
    printf("ESTOP. EtherCAT became degraded at %s.\n", time_str);
    printf("[EtherCAT Error] Stopping RT process.\n");
}

static int run_ethercat(const char *ifname)
{
    int i;
    int oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    num = 1;

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("[EtherCAT Init] Initialization on device %s succeeded.\n", ifname);
        /* find and auto-config slaves */

        if (ec_config_init(FALSE) > 0)
        {
            printf("[EtherCAT Init] %d slaves found and configured.\n", ec_slavecount);
            if (ec_slavecount < SLAVE_NUMBER)
            {
                printf("[RT EtherCAT] Warning: Expected %d slaves, found %d.\n", SLAVE_NUMBER, ec_slavecount);
            }

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
                ec_slave[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA;

            ec_config_map(&IOmap);
            ec_configdc();

            printf("[EtherCAT Init] Mapped slaves.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * SLAVE_NUMBER);

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
            {
                printf("[SLAVE %d]\n", slave_idx);
                printf("  IN  %d bytes, %d bits\n", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
                printf("  OUT %d bytes, %d bits\n", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
                printf("\n");
            }

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 8)
                oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 8)
                iloop = 8;

            printf("[EtherCAT Init] segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            printf("[EtherCAT Init] Requesting operational state for all slaves...\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("[EtherCAT Init] Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("[EtherCAT Init] Operational state reached for all slaves.\n");
                inOP = TRUE;
                return 1;
            }
            else
            {
                printf("[EtherCAT Error] Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        }
        else
        {
            printf("[EtherCAT Error] No slaves found!\n");
        }
    }
    else
    {
        printf("[EtherCAT Error] No socket connection on %s - are you running run.sh?\n", ifname);
    }
    return 0;
}

static int err_count = 0;
static int err_iteration_count = 0;
/**@brief EtherCAT errors are measured over this period of loop iterations */
#define K_ETHERCAT_ERR_PERIOD 100

/**@brief Maximum number of etherCAT errors before a fault per period of loop iterations */
#define K_ETHERCAT_ERR_MAX 20

static OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    (void)ptr;
    int slave = 0;
    while (1)
    {
        // count errors
        if (err_iteration_count > K_ETHERCAT_ERR_PERIOD)
        {
            err_iteration_count = 0;
            err_count = 0;
        }

        if (err_count > K_ETHERCAT_ERR_MAX)
        {
            // possibly shut down
            printf("[EtherCAT Error] EtherCAT connection degraded.\n");
            printf("[Simulink-Linux] Shutting down....\n");
            degraded_handler();
            break;
        }
        err_iteration_count++;

        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("[EtherCAT Error] Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                        err_count++;
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("[EtherCAT Error] Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                        err_count++;
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("[EtherCAT Status] Slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("[EtherCAT Error] Slave %d lost\n", slave);
                            err_count++;
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("[EtherCAT Status] Slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("[EtherCAT Status] Slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("[EtherCAT Status] All slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(50000);
    }
}

int EtherCAT_Init(char *ifname)
{
    int i;
    int rc;
    printf("[EtherCAT] Initializing EtherCAT\n");
    osal_thread_create((void *)&checkThread, 128000, (void *)&ecatcheck, (void *)&ctime);
    for (i = 1; i < 5; i++)
    {
        printf("[EtherCAT] Attempting to start EtherCAT, try %d of 4.\n", i);
        printf("ifname is %s ?\n", ifname);
        rc = run_ethercat(ifname);
        if (rc)
            break;
        osal_usleep(1000000);
    }
    if (rc)
        printf("[EtherCAT] EtherCAT successfully initialized on attempt %d \n", i);
    else
    {
        printf("[EtherCAT Error] Failed to initialize EtherCAT after 100 tries. \n");
    }
    return ec_slavecount;
}

static int wkc_err_count = 0;
static int wkc_err_iteration_count = 0;

// 数组大小根据从站数量确定
EtherCAT_Msg Rx_Message[SLAVE_NUMBER];
EtherCAT_Msg Tx_Message[SLAVE_NUMBER];
/**
 * @description:
 * @return {*}
 */
void EtherCAT_Run()
{
    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if (wkc_err_count > K_ETHERCAT_ERR_MAX)
    {
        printf("哈哈[EtherCAT Error] Error count too high!\n");
        degraded_handler();
    }
    // send
    ec_send_processdata();
    // receive
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    EtherCAT_Data_Get();
    //  check for dropped packet
    if (wkc < expectedWKC)
    {
        printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
        wkc_err_count++;
    }
    else
    {
        needlf = TRUE;
    }
    wkc_err_iteration_count++;
}

/**
 * @description: slave data get
 * @return {*}
 * @author: Kx Zhang
 */
void EtherCAT_Data_Get()
{
    for (int slave = 0; slave < ec_slavecount; ++slave)
    {
        EtherCAT_Msg *slave_src = (EtherCAT_Msg *)(ec_slave[slave + 1].inputs);
        if (slave_src)
            Rx_Message[slave] = *(EtherCAT_Msg *)(ec_slave[slave + 1].inputs);
        // uint8_t ack_status = RV_can_data_repack(&Rx_Message[slave], comm_ack, slave);
        printf("--------哈哈-------------------EtherCAT_Data_Get------------------------------\n");
        RV_can_data_repack(&Rx_Message[slave], comm_ack, slave);
    }
}
// *************

// *****************************
// *****************************
// *****************************
void EtherCAT_Get_State()
{
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    for (int slave = 0; slave < ec_slavecount; ++slave)
    {
        EtherCAT_Msg *slave_src = (EtherCAT_Msg *)(ec_slave[slave + 1].inputs);
        if (slave_src)
        {
            // printf("---------------------------EtherCAT_Get_State------------------------------\n");
            Rx_Message[slave] = *(EtherCAT_Msg *)(ec_slave[slave + 1].inputs);
        }
        if (slave < 2)
        {
            // printf("从站数量： %d\r\n", ec_slavecount);
            //  printf("------------slave is%d     \n", slave);
            RV_can_data_repack(&Rx_Message[slave], comm_ack, slave);
        }
        else
            RV_can_imu_data_repack(&Rx_Message[slave]);
        if (slave == 0)
        {
            // printf("---------------------------slave == 0------------------------------\n");
            for (int motor_index = 1; motor_index < 7; motor_index++) // motor_index=1 -> 3    2->4  3->5
            {
                motorDate_recv[motor_index - 1].pos_ = rv_motor_msg[motor_index].angle_actual_rad;
                motorDate_recv[motor_index - 1].vel_ = rv_motor_msg[motor_index].speed_actual_rad;
                motorDate_recv[motor_index - 1].tau_ = rv_motor_msg[motor_index].current_actual_float;
            }
        }
        else if (slave == 1)
        {
            // printf("---------------------------slave == 1------------------------------\n");
            for (int motor_index = 7; motor_index <= 12; motor_index++)
            {
                motorDate_recv[motor_index - 1].pos_ = rv_motor_msg[motor_index].angle_actual_rad;
                motorDate_recv[motor_index - 1].vel_ = rv_motor_msg[motor_index].speed_actual_rad;
                motorDate_recv[motor_index - 1].tau_ = rv_motor_msg[motor_index].current_actual_float;
            }
        }
        else if (slave == 2)
        {
            memcpy(imuData_recv.angle_float, imu_msg.angle_float, 3 * 4);
            memcpy(imuData_recv.gyro_float, imu_msg.gyro_float, 3 * 4);
            memcpy(imuData_recv.accel_float, imu_msg.accel_float, 3 * 4);
            memcpy(imuData_recv.mag_float, imu_msg.mag_float, 3 * 4);
            memcpy(imuData_recv.quat_float, imu_msg.quat_float, 4 * 4);
        }
    }

    // Revert_State((YKSMotorData *)motorDate_recv);

    // 扭矩常数=1 kp=15,kd=0.5,减速比=6
    //  check for dropped packet
    if (wkc < expectedWKC)
    {
        printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
        wkc_err_count++;
    }
    else
    {
        needlf = TRUE;
    }
    wkc_err_iteration_count++;
}

// CAN1
void disable_CAN1_MOTOR()
{
    Tx_Message[0].can_ide = 0;
    Tx_Message[0].motor[0].id = 3;
    Tx_Message[0].motor[1].id = 1;
    Tx_Message[0].motor[2].id = 2;
    Tx_Message[0].motor[3].id = 3;
    Tx_Message[0].motor[4].id = 1;
    Tx_Message[0].motor[5].id = 2;
    for (int index = 0; index < 6; index++)
    {
        Tx_Message[0].motor[index].rtr = 0;
        Tx_Message[0].motor[index].dlc = 8;
        Tx_Message[0].motor[index].data[0] = 0xff;
        Tx_Message[0].motor[index].data[1] = 0xff;
        Tx_Message[0].motor[index].data[2] = 0xff;
        Tx_Message[0].motor[index].data[3] = 0xff;
        Tx_Message[0].motor[index].data[4] = 0xff;
        Tx_Message[0].motor[index].data[5] = 0xff;
        Tx_Message[0].motor[index].data[6] = 0xff;
        Tx_Message[0].motor[index].data[7] = 0xfd;
    }
    EtherCAT_Msg *slave_dest_1 = (EtherCAT_Msg *)(ec_slave[1].outputs);
    if (slave_dest_1)
        *(EtherCAT_Msg *)(ec_slave[1].outputs) = Tx_Message[0];
    printf(" Youbot motors disable success!\n");
}

void enable_CAN1_MOTOR()
{
    Tx_Message[0].can_ide = 0;
    Tx_Message[0].motor[0].id = 3;
    Tx_Message[0].motor[1].id = 1;
    Tx_Message[0].motor[2].id = 2;
    Tx_Message[0].motor[3].id = 3;
    Tx_Message[0].motor[4].id = 1;
    Tx_Message[0].motor[5].id = 2;
    for (int index = 0; index < 6; index++)
    {
        Tx_Message[0].motor[index].rtr = 0;
        Tx_Message[0].motor[index].dlc = 8;
        Tx_Message[0].motor[index].data[0] = 0xff;
        Tx_Message[0].motor[index].data[1] = 0xff;
        Tx_Message[0].motor[index].data[2] = 0xff;
        Tx_Message[0].motor[index].data[3] = 0xff;
        Tx_Message[0].motor[index].data[4] = 0xff;
        Tx_Message[0].motor[index].data[5] = 0xff;
        Tx_Message[0].motor[index].data[6] = 0xff;
        Tx_Message[0].motor[index].data[7] = 0xfc;
    }
    EtherCAT_Msg *slave_dest_1 = (EtherCAT_Msg *)(ec_slave[1].outputs);
    if (slave_dest_1)
        *(EtherCAT_Msg *)(ec_slave[1].outputs) = Tx_Message[0];
    printf(" Youbot Can1 motors enable success! move carefully!\n");
}

// CAN2
void disable_CAN2_MOTOR()
{
    Tx_Message[1].can_ide = 0;
    Tx_Message[1].motor[0].id = 3;
    Tx_Message[1].motor[1].id = 1;
    Tx_Message[1].motor[2].id = 2;
    Tx_Message[1].motor[3].id = 3;
    Tx_Message[1].motor[4].id = 1;
    Tx_Message[1].motor[5].id = 2;
    for (int index = 0; index < 6; index++)
    {
        Tx_Message[1].motor[index].rtr = 0;
        Tx_Message[1].motor[index].dlc = 8;
        Tx_Message[1].motor[index].data[0] = 0xff;
        Tx_Message[1].motor[index].data[1] = 0xff;
        Tx_Message[1].motor[index].data[2] = 0xff;
        Tx_Message[1].motor[index].data[3] = 0xff;
        Tx_Message[1].motor[index].data[4] = 0xff;
        Tx_Message[1].motor[index].data[5] = 0xff;
        Tx_Message[1].motor[index].data[6] = 0xff;
        Tx_Message[1].motor[index].data[7] = 0xfd;
    }
    EtherCAT_Msg *slave_dest_2 = (EtherCAT_Msg *)(ec_slave[2].outputs);
    if (slave_dest_2)
        *(EtherCAT_Msg *)(ec_slave[2].outputs) = Tx_Message[1];
    ec_send_processdata();
    printf(" Youbot can2 motors disable success!\n");
}

void enable_CAN2_MOTOR()
{
    Tx_Message[1].can_ide = 0;
    Tx_Message[1].motor[0].id = 3;
    Tx_Message[1].motor[1].id = 1;
    Tx_Message[1].motor[2].id = 2;
    Tx_Message[1].motor[3].id = 3;
    Tx_Message[1].motor[4].id = 1;
    Tx_Message[1].motor[5].id = 2;
    for (int index = 0; index < 6; index++)
    {
        Tx_Message[1].motor[index].rtr = 0;
        Tx_Message[1].motor[index].dlc = 8;
        Tx_Message[1].motor[index].data[0] = 0xff;
        Tx_Message[1].motor[index].data[1] = 0xff;
        Tx_Message[1].motor[index].data[2] = 0xff;
        Tx_Message[1].motor[index].data[3] = 0xff;
        Tx_Message[1].motor[index].data[4] = 0xff;
        Tx_Message[1].motor[index].data[5] = 0xff;
        Tx_Message[1].motor[index].data[6] = 0xff;
        Tx_Message[1].motor[index].data[7] = 0xfc;
    }
    EtherCAT_Msg *slave_dest_2 = (EtherCAT_Msg *)(ec_slave[2].outputs);
    if (slave_dest_2)
        *(EtherCAT_Msg *)(ec_slave[2].outputs) = Tx_Message[1];
    ec_send_processdata();
    printf(" Youbot motors enable success! move carefully!\n");
}

// Tao code end
void EtherCAT_Send_Command(YKSMotorData *mot_data)
{

    // uint16_t motor_id;
    uint16_t slave;

    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if (wkc_err_count > K_ETHERCAT_ERR_MAX)
    {
        printf("[look:EtherCAT Error] Error count too high!\n");
        degraded_handler();
    }
    for (int index = 0; index < 12; index++)
    {
        if (index < 6)
        {
            slave = 0;
            send_motor_ctrl_cmd(&Tx_Message[slave], index + 1, mot_data[index].kp_, mot_data[index].kd_, mot_data[index].pos_des_, mot_data[index].vel_des_, mot_data[index].ff_);
        }
        else if (index < 12)
        {
            slave = 1;
            send_motor_ctrl_cmd(&Tx_Message[slave], index + 1 - 6, mot_data[index].kp_, mot_data[index].kd_, mot_data[index].pos_des_, mot_data[index].vel_des_, mot_data[index].ff_);
        }
        if (index == 5 || index == 11)
        {
            EtherCAT_Msg *slave_dest = (EtherCAT_Msg *)(ec_slave[slave + 1].outputs);
            if (slave_dest)
                *(EtherCAT_Msg *)(ec_slave[slave + 1].outputs) = Tx_Message[slave];
        }
    }
    ec_send_processdata();
}

void runImpl()
{
    std::cout << "hahaha" << std::endl;
    while (running)
    {
        EtherCAT_Run();
    }
}

void startRun()
{
    std::cout << "xixixi" << std::endl;
    running = true;
    runThread = std::thread(runImpl);
}
