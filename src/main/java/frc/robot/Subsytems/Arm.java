// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.ControllerFactory;

public class Arm extends SubsystemBase {

  /** Creates a new Arm. */

  public CANSparkMax mainMotor;
 // public final RelativeEncoder encoder;

  //private WPI_VictorSPX slaveMotor;

  //public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  //public double topP, topI, topD, topIz, topFF;
  
  private SparkMaxPIDController m_pidController;
  public RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  

  public Arm() {
    mainMotor = ControllerFactory.sparkMAX(Constants.sparkMAX_ARM, false);
    //slaveMotor = ControllerFactory.victorSPX(Constants.victorSPX_ARM, false);
    //slaveMotor.setNeutralMode(NeutralMode.Coast);
    //encoder = mainMotor.getEncoder();
    //encoder.setPositionConversionFactor(0);
    mainMotor.setIdleMode(IdleMode.kBrake);
    // initialize motor

    // initialze PID controller and encoder objects
    m_pidController = mainMotor.getPIDController();
    m_encoder = mainMotor.getEncoder();
    m_encoder.setPosition(0);

    // PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("ARM P Gain", kP);
    SmartDashboard.putNumber("ARM I Gain", kI);
    SmartDashboard.putNumber("ARM D Gain", kD);
    SmartDashboard.putNumber("ARM I Zone", kIz);
    SmartDashboard.putNumber("ARM Feed Forward", kFF);
    SmartDashboard.putNumber("ARM Max Output", kMaxOutput);
    SmartDashboard.putNumber("ARM Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("ARM Max Velocity", maxVel);
    SmartDashboard.putNumber("ARM Min Velocity", minVel);
    SmartDashboard.putNumber("ARM Max Acceleration", maxAcc);
    SmartDashboard.putNumber("ARM Allowed Closed Loop Error", allowedErr);


    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("ARM Mode", true);
  }

  public void moveArm(double speed){
    // read PID coefficients from SmartDashboard
    //feed arm
    double kMeasuredPosHorizontal = SmartDashboard.getNumber("ARM Horizontal Position: ", 42); //Position measured when arm is horizontal
    double kTicksPerDegree = 360/42; //Sensor is 1:1 with arm rotation
    double currentPos = m_encoder.getPosition();
    SmartDashboard.putNumber("ARM Position: ", currentPos); 
    double degrees = (currentPos - kMeasuredPosHorizontal) * kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

    double maxGravityFF = 0.07;
    double ff = cosineScalar * maxGravityFF;
    m_pidController.setFF(ff); kFF = ff; 

    //REAL STUFF
    // if(currentPos >= SmartDashboard.getNumber("ARM Resting position ", 0) && currentPos <= kMeasuredPosHorizontal)
    //   m_pidController.setReference(speed, CANSparkMax.ControlType.kSmartMotion);


    // SmartDashBoard
    double p = SmartDashboard.getNumber("ARM P Gain", 0);
    double i = SmartDashboard.getNumber("ARM I Gain", 0);
    double d = SmartDashboard.getNumber("ARM D Gain", 0);
    double iz = SmartDashboard.getNumber("ARM I Zone", 0);
    ff = SmartDashboard.getNumber("ARM Feed Forward", 0);

    
    double max = SmartDashboard.getNumber("ARM Max Output", 0);
    double min = SmartDashboard.getNumber("ARM Min Output", 0);
    double maxV = SmartDashboard.getNumber("ARM Max Velocity", 0);
    double minV = SmartDashboard.getNumber("ARM Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("ARM Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("ARM Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
      SmartDashboard.putNumber("ARM SPEED: ", speed);
      //slaveMotor.set(ControlMode.PercentOutput, speed);

      mainMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
