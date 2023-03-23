// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.ControllerFactory;
import frc.robot.Util.PID;

public class DriveTrain extends SubsystemBase {

	  public double turnMultiplier;
  
    public WPI_VictorSPX leftSlaveMotor  = null;
    public WPI_TalonSRX leftMasterMotor = null;

    public WPI_VictorSPX rightSlaveMotor = null;
    public WPI_TalonSRX rightMasterMotor  = null;

    private final PID arcadePID;
    //private boolean encoderUse;

    //final DifferentialDrive diffDrive;
  
  public DriveTrain() {
    turnMultiplier = 0.5;
    //encoderUse = false;

    /* Creates a new DriveTrain. */
    leftMasterMotor = ControllerFactory.talonSRX(Constants.talonSRX_LEFT, true);
    leftSlaveMotor = ControllerFactory.victorSPX(Constants.victorSPX_LEFT, true, leftMasterMotor);

    rightMasterMotor = ControllerFactory.talonSRX(Constants.talonSRX_RIGHT, false);
    rightSlaveMotor = ControllerFactory.victorSPX(Constants.victorSPX_RIGHT, false, rightMasterMotor);

    //diff drive
    //diffDrive = new DifferentialDrive(leftMasterMotor, rightMasterMotor);

    rightMasterMotor.set(ControlMode.PercentOutput, 0);
    rightMasterMotor.set(ControlMode.PercentOutput, 0);

    //PID settings
    final double kP = 0.15;
    final double kI = 0.000009;
    final double kD = 0.0065;
    final double MAX_PID_DRIVE = 0.5;
    final double MIN_PID_DRIVE = -0.5;
    arcadePID = new PID(kP, kI, kD);
    arcadePID.setOutputLimits(MIN_PID_DRIVE, MAX_PID_DRIVE);
  }
  public void test(double moveSpeed, double turnSpeed){
    moveSpeed /= -4;
    rightMasterMotor.set(ControlMode.PercentOutput, moveSpeed);
  }

  public void arcadeDrive(double moveSpeed, double turnSpeed){
    //turnSpeed = turnSpeed * turnMultiplier;
    //Drives the robot using arcade drive
    //double max =Math.max(Math.abs(moveSpeed), Math.abs(turnSpeed));
    double left =(moveSpeed + turnSpeed);
    double right =(moveSpeed - turnSpeed);
    
    leftMasterMotor.set(ControlMode.PercentOutput, left); 
    rightMasterMotor.set(ControlMode.PercentOutput, right); 

    SmartDashboard.putNumber("DRIVE ARCADE left: ", left);
    SmartDashboard.putNumber("DRIVE ARCADE right:", right);
  }

  public void tankDrive(double left, double right){
    //Drives the robot using arcade drive
    //double max =Math.max(Math.abs(moveSpeed), Math.abs(turnSpeed));

    leftMasterMotor.set(ControlMode.PercentOutput, left); 
    rightMasterMotor.set(ControlMode.PercentOutput, right); 

    SmartDashboard.putNumber("DRIVE TANK left: ", left);
    SmartDashboard.putNumber("DRIVE TANK right: ", right);
  }

 
}
