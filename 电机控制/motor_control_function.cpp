# 源码文件
/**
 * @brief Controls the motor based on the given control flag and distance.
 *
 * @param controlFlag An integer flag that determines the control action:
 *                    - 1: Enable the motor and move it by the specified relative distance.
 *                    - 2: Stop the motor.
 *                    - 3: Disable the motor.
 *                    - 4: Move the motor at a specified speed.
 * @param distance The distance to move the motor, used only when controlFlag is 1.
 *                 This value is converted from degrees to steps.
 * @return int8_t Always returns 0.
 */
int8_t patchclamp::motor_control(int8 controlFlag, double distance)
{
    static int8_t last_control_mode = 0;
    switch (controlFlag)
    {
    case 1:
        motor_driver.ControlMode = 1;
        if (!motor_driver.motor_enabled)
        {
            if (device)
            {
                device->enable();                                                        // 使能驱动器
                device->cfgMcs(motor_driver.MscParam);                                   // 设置细分参数。默认是16细分，msc=4
                device->cfgSpd(motor_driver.RunningSpd);                                 // 设置默认速度
                stepcal = 0;                                                             // 清空步数计数器
                motor_driver.motor_enabled = true;                                       // 确认之后 再修改值
                device->autoStsOn();                                                     // 自动状态更新
                qDebug() << "motor enabled, Mcs Param" << motor_driver.MscParam << endl; // 输出日志
            }
        }
        if (motor_driver.ControlMode != last_control_mode)//当模式发生了改变，重新设置细分和速度参数
        {
            device->cfgMcs(motor_driver.MscParam); //重新设置 msc 参数
            device->cfgSpd(5 * 200.0f * pow(2, motor_driver.MscParam) ); // 切换到默认转速
            last_control_mode = motor_driver.ControlMode; //更新值
        }
        if (device)
        {
            //            device->cfgMcs(mcsmotor); // 设置细分
            if(device->isOnline() == false)
            {
                qDebug() << "Fail to Turn.device is offline, please check the connection." << endl;
                device->online(); // 设置为在线，重新参与通讯
                device->autoStsOn(); // 自动状态更新
                return -1; // 如果设备离线，返回错误
            }
            motor_driver.motor_isRunning = true;                                                             // 标记运行中
            device->rmove(static_cast<int32_t>(distance * pow(2, motor_driver.MscParam) * 200.0 / (360.0))); // 转角度为步数并以相对位置模式移动
            motor_driver.distanceCnt += distance;                                                            // 记录转动角度
            emit sendPumpStete(2, 0);                                                                        // 定向转动状态
        }
        // 储存转动角度并在状态栏显示
        stepcal += distance;
        sendState(QString("motor angle : %03.1f° ").arg(stepcal));
        break;
    case 2:
        motor_driver.ControlMode = 2;
        if (device)
        {
            if(isMotorBusy())
            {
                qDebug() << "Fail to Stop.motor is busy, please wait for the current operation to finish." << endl;
                return -1; // 如果电机忙碌，返回错误
            }
            if(device->isOnline() == false)
            {
                qDebug() << "Fail to Stop.device is offline, please check the connection." << endl;
                device->online(); // 设置为在线，重新参与通讯
                device->autoStsOn(); // 自动状态更新
                return -1; // 如果设备离线，返回错误
            }
            device->stop(1);//立刻停止
            motor_driver.motor_isRunning = false; // 标记停止状态
            last_control_mode = motor_driver.ControlMode; //更新值
            emit sendPumpStete(0, 0);             // 发送停止信号
        }
        break;
    case 3:
        motor_driver.ControlMode = 3;
        if (device)
        {
            device->stop(1); // 立刻停止
            device->disable();// 禁用驱动器
            motor_driver.motor_enabled = false;   // 标记关闭使能
            motor_driver.motor_isRunning = false; // 标记运行中
            last_control_mode = motor_driver.ControlMode; //更新值
            emit sendPumpStete(0, 0);             // 发送停止信号
        }
        break;
    case 4: // 指定速度转定的模式
        motor_driver.ControlMode = 4;
        if (!motor_driver.motor_enabled)
        {
            if (device)
            {
                device->enable();                                                        // 使能驱动器
                device->cfgMcs(motor_driver.MscParam);                                   // 配置细分参数
                device->cfgSpd(motor_driver.RunningSpd);                                 // 设置默认速度
                motor_driver.motor_enabled = true;                                       // 标记已使能并做了初始化修改
                device->autoStsOn();                                                     // 自动状态更新
                qDebug() << "motor enabled, Mcs Param" << motor_driver.MscParam << endl; // 输出日志
            }
        }
        if (motor_driver.ControlMode != last_control_mode)
        {
            device->cfgMcs(motor_driver.MscParam); //设置细分参数
            device->cfgSpd(motor_driver.RunningSpd); // 修正为动态的变量值
            last_control_mode = motor_driver.ControlMode;
        }
        if (device)
        {
             if(device->isOnline() == false)
            {
                qDebug() << "Fail to Turn at specific Speed.device is offline, please check the connection." << endl;
                device->online(); // 设置为在线，重新参与通讯
                device->autoStsOn(); // 自动状态更新
                return -1; // 如果设备离线，返回错误
            }
            /***当速度值发生改变时，会写入驱动器。无需在此处显式调用 ** */
            motor_driver.motor_isRunning = true;                  // 标记运行中
            emit sendPumpStete(1, motor_driver.RunningSpd_Angle); // 发送转动速度
            device->move();                                       // 按照指定的速度进行转动
        }
        break;
    default:
        break;
    }
    return 0;
}


