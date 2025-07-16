/**
 * @brief Set the motor speed.
 *
 * This function sets the speed of the motor in degrees per second.
 * It converts the speed from degrees per second to pulses per second
 * and configures the device with the calculated speed.
 *
 * @param speed The speed in degrees per second.
 */
void patchclamp::motor_config_speed(double speed) // 单位是Degree/s
{
    static double last_speed = 0;
    if (device)
    {
        // 将角度每秒转换为脉冲数/S
        if (last_speed != speed)
        {
            double pulsePerSecond = speed * (200 * pow(2, motor_driver.MscParam) / 360.0); // 转换为步数
            motor_driver.RunningSpd = pulsePerSecond;                                      // 每秒的脉冲数量
            motor_driver.RunningSpd_Angle = speed;                                         // 每秒的角度
            device->cfgSpd(motor_driver.RunningSpd);                                       // 将速度值写入驱动器
            last_speed = speed;                                                            // 记录最新写入的值
            // qDebug() << "Rotate Speed change to " << speed << "Deg/S" << endl;             // 输出日志
            if (motor_driver.motor_isRunning)
            {
                emit sendPumpStete(1, motor_driver.RunningSpd_Angle); // 更新转动速度
            }
        }
    }
}
