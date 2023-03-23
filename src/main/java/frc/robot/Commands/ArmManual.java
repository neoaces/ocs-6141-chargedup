package frc.robot.Commands;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Subsytems.Arm;

public class ArmManual extends CommandBase {

  public ArmManual() {
    addRequirements(Robot.driveTrain);
  }

  Arm arm;
  boolean stop = false;

  public void initialize() {
    arm = new Arm();
  }

  public  void execute() {
      if(!Robot.input.joystick.getRawButton(7))
        arm.moveArm(Robot.input.joystick.getY());
      else
        arm.moveArm(Robot.input.joystick.getY()/2);
  

    stop = Robot.input.joystick.getRawButton(10);
  }

  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }
}

