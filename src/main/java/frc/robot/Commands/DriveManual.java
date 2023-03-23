// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Input;
import frc.robot.Robot;



public class DriveManual extends CommandBase {

  public double speedFactor = 2;
  boolean driveMode = false;

  public DriveManual() {
    addRequirements(Robot.driveTrain);
  }

  public void initialize() {
    Robot.driveTrain.leftMasterMotor.setNeutralMode(NeutralMode.Brake);
    Robot.driveTrain.rightMasterMotor.setNeutralMode(NeutralMode.Brake);
    Robot.driveTrain.leftSlaveMotor.setNeutralMode(NeutralMode.Brake);
    Robot.driveTrain.rightMasterMotor.setNeutralMode(NeutralMode.Brake);
  }

  public  void execute() {

    if(Robot.input.controller.getBButtonPressed()){
      Robot.driveTrain.leftMasterMotor.setNeutralMode(NeutralMode.Brake);
      Robot.driveTrain.rightMasterMotor.setNeutralMode(NeutralMode.Brake);
      Robot.driveTrain.leftSlaveMotor.setNeutralMode(NeutralMode.Brake);
      Robot.driveTrain.rightMasterMotor.setNeutralMode(NeutralMode.Brake);
    }
    if(Robot.input.controller.getAButtonPressed()){
      if(driveMode == false)
        driveMode = true;
      else
        driveMode = false;
    }
    if(driveMode){
      double left = Robot.input.controller.getLeftY()/speedFactor;
      double right = Robot.input.controller.getLeftX()/speedFactor;
      if(Robot.input.controller.getRightBumper())
        Robot.driveTrain.arcadeDrive(left*2, right*2);
      else
        Robot.driveTrain.arcadeDrive(left, right);
    }else{
      double left = Robot.input.controller.getLeftY()/speedFactor;
      double right = Robot.input.controller.getRightY()/speedFactor;
      if(Robot.input.controller.getRightBumper())
        Robot.driveTrain.tankDrive(left*2, right*2);
      else
        Robot.driveTrain.tankDrive(left, right);
    }
    
  }

  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }
}