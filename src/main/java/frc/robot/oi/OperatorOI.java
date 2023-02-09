package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OperatorOI extends OIBase {
	/*
	
	A - stop intake
	B - shoot intake
	X - run intake
	Y

	Start(left)
	Back(right)

	LB
	RB
	
	LT
	RT

	LS
	RS
	
	LS Click
	RS Click

	*/
	public OperatorOI(XboxController controller) {
		super(controller);
	}

	public Trigger getRunIntakeButton() {
		return new JoystickButton(this.controller, XboxController.Button.kX.value);
	}

    public Trigger getStopIntakeButton() {
		return new JoystickButton(this.controller, XboxController.Button.kA.value);
	}

    public Trigger getShootIntakeButton() {
		return new JoystickButton(this.controller, XboxController.Button.kB.value);
	}
}
