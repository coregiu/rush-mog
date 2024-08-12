#include <attitude_executor.h>

struct gimbal_info gimbal_info = {0};
struct gimbal_info pre_gimbal_info = {0};
int32_t gimbale_data_buffer[6] = {0};

struct command_context servo_cmd = {COMMAND_ADAPTE_SERVO, MODULE_ROBOOT, 0, DELAY_BEFOR_EXE, COMMAND_TYPE_AUTO};

void init_attitude_module()
{
    // bind_dma1_to_iic2();
    uint8_t result = mpu_init();
    if (result != 0)
    {
        uart_log_string_no_enter("mpu init error: ");
        uart_log_number(result);
        uart_log_enter_char();
        return;
    }
}

uchar compare_gimbal_info(struct gimbal_info *pre_gimbal_info, struct gimbal_info *gimbal_info)
{
    if ((int)pre_gimbal_info->roll != (int)gimbal_info->roll)
    {
        return 1;
    }
    if ((int)pre_gimbal_info->pitch != (int)gimbal_info->pitch)
    {
        return 1;
    }
    if ((int)pre_gimbal_info->yaw != (int)gimbal_info->yaw)
    {
        return 1;
    }
    return 0;
}

void set_gimbal_info(struct gimbal_info *pre_gimbal_info, struct gimbal_info *gimbal_info)
{
    pre_gimbal_info->gyro_x_raw = gimbal_info->gyro_x_raw;
    pre_gimbal_info->gyro_y_raw = gimbal_info->gyro_y_raw;
    pre_gimbal_info->gyro_z_raw = gimbal_info->gyro_z_raw;

    pre_gimbal_info->accl_x_raw = gimbal_info->accl_x_raw;
    pre_gimbal_info->accl_y_raw = gimbal_info->accl_y_raw;
    pre_gimbal_info->accl_z_raw = gimbal_info->accl_z_raw;

    pre_gimbal_info->gyro_x = gimbal_info->gyro_x;
    pre_gimbal_info->gyro_y = gimbal_info->gyro_y;
    pre_gimbal_info->gyro_z = gimbal_info->gyro_z;

    pre_gimbal_info->accl_x = gimbal_info->accl_x;
    pre_gimbal_info->accl_y = gimbal_info->accl_y;
    pre_gimbal_info->accl_z = gimbal_info->accl_z;

    pre_gimbal_info->roll = gimbal_info->roll;
    pre_gimbal_info->pitch = gimbal_info->pitch;
    pre_gimbal_info->yaw = gimbal_info->yaw;
    pre_gimbal_info->temperature = gimbal_info->temperature;
}

void log_gimbal_info(struct gimbal_info *gimbal_info)
{
    uart_log_string_no_enter("G_X:");
    uart_log_number(gimbal_info->gyro_x_raw);
    uart_log_string_no_enter("|G_Y:");
    uart_log_number(gimbal_info->gyro_y_raw);
    uart_log_string_no_enter("|G_Z:");
    uart_log_number(gimbal_info->gyro_z_raw);
    uart_log_string_no_enter("|A_X:");
    uart_log_number(gimbal_info->accl_x_raw);
    uart_log_string_no_enter("|A_Y:");
    uart_log_number(gimbal_info->accl_y_raw);
    uart_log_string_no_enter("|A_Z:");
    uart_log_number(gimbal_info->accl_z_raw);
    uart_log_string_no_enter("|M_X:");
    uart_log_number(gimbal_info->magn_x_raw);
    uart_log_string_no_enter("|M_Y:");
    uart_log_number(gimbal_info->magn_y_raw);
    uart_log_string_no_enter("|M_Z:");
    uart_log_number(gimbal_info->magn_z_raw);
    uart_log_string_no_enter("|roll:");
    uart_log_number(gimbal_info->roll);
    uart_log_string_no_enter("|pitch:");
    uart_log_number(gimbal_info->pitch);
    uart_log_string_no_enter("|yaw:");
    uart_log_number(gimbal_info->yaw);
    uart_log_string_no_enter("|Temp:");
    uart_log_number(gimbal_info->temperature);
    uart_log_enter_char();
}

void update_attitude_state(struct command_context *command_context)
{
    uchar result = mpu_get_gyroscope(&gimbal_info.gyro_x_raw, &gimbal_info.gyro_y_raw, &gimbal_info.gyro_z_raw);
    result |= mpu_get_accelerometer(&gimbal_info.accl_x_raw, &gimbal_info.accl_y_raw, &gimbal_info.accl_z_raw);
    gimbal_info.temperature = mpu_get_temperature();

    if (result != 0)
    {
        uart_log_string_data("mpu read error");
        return;
    }

    if (mpu_type == MPU_6050 || mpu_type == MPU_6500)
    {
        compute_angle(&gimbal_info);
    }
    else if (mpu_type == MPU_9250)
    {
        result = mpu_get_magnetometer(&gimbal_info.magn_x_raw, &gimbal_info.magn_y_raw, &gimbal_info.magn_z_raw);
        if (result != 0)
        {
            uart_log_string_data("magn read error");
            return;
        }
        mpu_compute_mag(&gimbal_info.magn_x_raw, &gimbal_info.magn_y_raw, &gimbal_info.magn_z_raw, &gimbal_info.magn_x, &gimbal_info.magn_y, &gimbal_info.magn_z);
        ahrs_update(&gimbal_info);
    }
    else
    {
        uart_log_string_data("nothing of mpu type!");
        return;
    }

    log_gimbal_info(&gimbal_info);
    if (compare_gimbal_info(&pre_gimbal_info, &gimbal_info) == 0)
    {
        // 位置没有变化
        return;
    }
    set_gimbal_info(&pre_gimbal_info, &gimbal_info);

    execute_command(&servo_cmd);
}

const struct module_command_executor attitude_executor = {init_attitude_module, update_attitude_state};