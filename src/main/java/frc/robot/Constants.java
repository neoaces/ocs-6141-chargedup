// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Constants {

    public static final int victorSPX_LEFT = 4;
    public static final int victorSPX_RIGHT = 3;
    public static final int talonSRX_LEFT = 10;
    public static final int talonSRX_RIGHT = 11;
    //public static final int victorSPX_ARM = 4;
    public static final int victorSPX_CLAW = 1;
    public static final int sparkMAX_ARM = 2;


    public static final int timeoutMs = 30;
    public static final double driveNeutralDeadband = 0.1;

    //public static final int joystickPort = 1;
    public static final int controllerPort = (int) SmartDashboard.getNumber("Controller Port: ", 1);
    public static final int joystickPort = (int) SmartDashboard.getNumber("Joystick Port: ", 0);

   // Smartdashboard.putnumber("DRIVETRAIN_LEFT_MOTOR_VICTORSPX1" , DRIVETRAIN_LEFT_MOTOR_VICTORSPX1);

}
