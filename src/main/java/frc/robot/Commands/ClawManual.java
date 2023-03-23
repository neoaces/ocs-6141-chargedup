package frc.robot.Commands;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Subsytems.Claw;

public class ClawManual extends CommandBase {

  public ClawManual() {
    addRequirements(Robot.driveTrain);
  }

  Claw claw;

  public void initialize() {
    claw = new Claw();
  }

  public  void execute() {

    double speed = Robot.input.joystick.getZ();
    if( speed > -0.3)
      claw.motor.set(ControlMode.PercentOutput, -speed/2);

    if(Robot.input.joystick.getRawButton(2)) {
        claw.motor.setNeutralMode(NeutralMode.Brake);
        claw.motor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }
}
