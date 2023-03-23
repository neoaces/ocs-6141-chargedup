// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Input;
import frc.robot.Robot;



public class DriveAutonomous extends CommandBase {

  public boolean fast = false;
  private ClawManual clawManual = new ClawManual();
  private ArmManual armManual = new ArmManual();
  Accelerometer accelerometer;
  LinearFilter xAccelFilter;

  public DriveAutonomous() {
    addRequirements(Robot.driveTrain);
  }

  public void initialize() {
    Robot.driveTrain.leftMasterMotor.setNeutralMode(NeutralMode.Brake);
    Robot.driveTrain.rightMasterMotor.setNeutralMode(NeutralMode.Brake);
    Robot.driveTrain.leftSlaveMotor.setNeutralMode(NeutralMode.Brake);
    Robot.driveTrain.rightMasterMotor.setNeutralMode(NeutralMode.Brake);
    Robot.gyro.setYawAxis(IMUAxis.kX);
    xAccelFilter = LinearFilter.movingAverage(10);
    accelerometer = new BuiltInAccelerometer();
  }

  public void execute() {
   
    double left = Robot.input.controller.getLeftY();
    double right = Robot.input.controller.getRightY();
    Robot.driveTrain.tankDrive(left, right);
  }

  public void routine(Timer timer) {

      if(timer.get() < 15){
        if(timer.get() < 1){
          Robot.driveTrain.tankDrive(0.6, 0.6);//---
        }else if(timer.get() < 4){
          Robot.driveTrain.tankDrive(-0.5, -0.5);//---
        }else if(timer.get() < 5){
          Robot.driveTrain.tankDrive(0.1, 0.1);//---
        }
      }

      // if(timer.get() < 15){
      //   if(timer.get() < 1){
      //     Robot.driveTrain.tankDrive(0.6, 0.6);//---
      //   }else if(timer.get() < 3){
      //     Robot.driveTrain.tankDrive(-0.7, -0.7);//---
      //   }else if(timer.get() < 5){
      //     Robot.driveTrain.tankDrive(-0.3, -0.3);//---
      //   }
      // }


    //openClaw();
    //closeClaw();

    // moveBackward(); time 
    // moveLine(timer.get(), timer, 4.78, -0.7);
    // openClaw(); time
    // closeClaw(); time
    //spin180();
    // moveLine(timer.get(), timer, 4.78, -0.6);
    // raiseArm();
    // openClaw(); time
    // moveLine(timer.get(), timer, 0.38, 0.4);
    // spin90Left();
    // moveLine(timer.get(), timer, 1.17, -0.6);
    // spin90Left();
    // engage(timer.get(), timer, 1.65, -0.7);

  }

  public void moveBackward(){
    Robot.driveTrain.tankDrive(-0.1, -0.1);//---
  }

  public void moveLine(double initialTime, Timer timer, double distance, double direction){
    double filteredZAccel = xAccelFilter.calculate(accelerometer.getZ())/9.8;
    double innitialSpeed =  0;
    double speed =  0;
    double displacment = 0;
    double time = 0; 

    System.out.println(filteredZAccel);
    while(displacment < distance){
      time = timer.get() - initialTime;
      speed = innitialSpeed * time + (filteredZAccel/2 * Math.pow(time, 2));
      displacment = speed * time;
      Robot.driveTrain.tankDrive(direction, direction);//---
    }
  }

  public void closeClaw(){
    clawManual.claw.motor.set(ControlMode.PercentOutput, -0.1);//---
  }

  public void openClaw(){
    clawManual.claw.motor.set(ControlMode.PercentOutput, 0.1);//---
  }

  public void raiseArm(){
    while(armManual.arm.m_encoder.getPosition() < 20){//---
      armManual.arm.mainMotor.set(-0.1);//---
    }
  }

  public void spin180 (){
    while(Robot.gyro.getAngle() < 180 ){//---
      Robot.driveTrain.tankDrive(-0.1, 0.1);//---
    }
  }

  public void spin90Left (){
    while(Robot.gyro.getAngle() < 90 ){//---
      Robot.driveTrain.tankDrive(0.1, -0.1);//---
    }
  }

  public void engage(double initialTime, Timer timer, double distance, double direction){
    double anglex = Robot.gyro.getAngle() - 90;

    double filteredZAccel = xAccelFilter.calculate(accelerometer.getZ())/9.8;
    double innitialSpeed =  0;
    double speed =  0;
    double displacment = 0;
    double time = 0; 

    System.out.println(filteredZAccel);
    while(displacment < distance){
      time = timer.get() - initialTime;
      speed = innitialSpeed * time + (filteredZAccel/2 * Math.pow(time, 2));
      displacment = speed * time;
      Robot.driveTrain.tankDrive(direction, direction);//---
    }
    while(displacment >= distance){
      if(anglex > 4 && anglex < 356){
        double adjust = -0.01 * (anglex);
        Robot.driveTrain.tankDrive(adjust, adjust);//---
        //moveLine(timer.get(), timer, distance, 0.4);
      }
    }
  }

  public void autonomous(double speed, double turn){
    
    Robot.driveTrain.arcadeDrive(speed, turn);
  }


  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }
}