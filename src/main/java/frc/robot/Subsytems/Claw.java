// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.ControllerFactory;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */

  public VictorSPX motor;


  public Claw() {
    motor = ControllerFactory.victorSPX(Constants.victorSPX_CLAW, false);
    //motor.setNeutralMode(NeutralMode.Brake);
  }

  public void move(double speed){
    motor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
