package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class Input {

    public XboxController controller = new XboxController(Constants.controllerPort);
    public Joystick joystick = new Joystick(Constants.joystickPort);
    public Input() {
		// joystick

	}

    public void execute(){
       
    }



    
    private void configureButtonBindings() {
        // new JoystickButton(controller, Button.kA.value)
        // .whenPressed(new InstantCommand(drivetrain::startCompressor, drivetrain));
        //new JoystickButton(controller, Button.kBumperRight.value)
        //.whenHeld(shoot); 
    
    }
    
}
