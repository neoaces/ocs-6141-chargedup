// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
//import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Commands.ArmManual;
import frc.robot.Commands.ClawManual;
import frc.robot.Commands.DriveAutonomous;
import frc.robot.Commands.DriveManual;
import frc.robot.Subsytems.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  public static DriveTrain driveTrain = new DriveTrain();
  public static Input input = new Input();
  public static ADIS16470_IMU gyro = new ADIS16470_IMU();
  public final DriveManual driveManualCommand = new DriveManual();
  public final DriveAutonomous driveAutonomousCommand = new DriveAutonomous();
  public final ArmManual ArmManualCommand = new ArmManual();
  public final ClawManual ClawManualCommand = new ClawManual();
  private Timer timer = new Timer();

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    driveAutonomousCommand.initialize();
    timer.start();
    timer.reset();
    Robot.gyro.calibrate();
  }
  @Override
  public void autonomousPeriodic() {
    driveAutonomousCommand.routine(timer);
  }

  @Override
  public void teleopInit() {
    driveManualCommand.initialize();
    ArmManualCommand.initialize();
    ClawManualCommand.initialize();
  }

  @Override
  public void teleopPeriodic() {
    driveManualCommand.execute();
    ArmManualCommand.execute();
    ClawManualCommand.execute();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
