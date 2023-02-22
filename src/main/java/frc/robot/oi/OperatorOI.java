package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OperatorOI extends OIBase {
	/*
	
	A - elevator down
	B - elevator up
	X 
	Y

	Start(left)
	Back(right)

	LB
	RB
	
	LT
	RT

	LS up and down controls elevator, right and left controls arm 
		(probably most movement for these subsystems will be to predesignated positions)
	RS
	
	LS Click
	RS Click

	POV for arm positions - high, middle, low, in
	*/
	public OperatorOI(XboxController controller) {
		super(controller);
	}

	public DoubleSupplier getElevatorSupplier() {
		return () -> this.controller.getLeftY();
	}

	public DoubleSupplier getArmSupplier() {
		return () -> this.controller.getLeftX();
	}

	public Trigger getHighArm(){
		return new Trigger(() -> this.controller.getPOV() == 0);
	}

	public Trigger getMidArm(){
		return new Trigger(() -> this.controller.getPOV() == 90);
	}

	public Trigger getLowArm(){
		return new Trigger(() -> this.controller.getPOV() == 180);
	}

	public Trigger getArmIn(){
		return new Trigger(() -> this.controller.getPOV() == 270);
	}

	public Trigger getElevatorUp(){
		return new JoystickButton(this.controller, XboxController.Button.kB.value);
	}

	public Trigger getElevatorDown(){
		return new JoystickButton(this.controller, XboxController.Button.kA.value);
	}
}
