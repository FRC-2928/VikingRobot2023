package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OperatorOI extends OIBase {
	/*

	A - stop intake
	B - shoot intake
	X - run intake
	Y - Initialize Elevator

	Start(left)
	Back(right)

	LB
	RB

	LT
	RT

	LS up and down controls elevator
		(probably most movement for these subsystems will be to predesignated positions)
	RS up and down controls arm

	LS Click
	RS Click

	potentially POV for arm positions - high, middle, ground, in
		

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

	public Trigger getInitializeElevatorButton() {
		return new JoystickButton(this.controller, XboxController.Button.kY.value);
	}

	public DoubleSupplier getElevatorSupplier() {
		return () -> this.controller.getLeftY();
	}

	public DoubleSupplier getArmSupplier() {
		return () -> this.controller.getRightY();
	}

	public Trigger getArmHigh() {
		return new Trigger(() -> this.controller.getPOV() == 0);
	}

	public Trigger getArmMid() {
		return new Trigger(() -> this.controller.getPOV() == 90);
	}

	public Trigger getArmLow() {
		return new Trigger(() -> this.controller.getPOV() == 180);
	}

	public Trigger getArmIn() {
		return new Trigger(() -> this.controller.getPOV() == 270);
	}

	public Trigger getElevatorUp(){
		return new JoystickButton(this.controller, XboxController.Button.kRightBumper.value);
	}

	public Trigger getElevatorDown(){
		return new JoystickButton(this.controller, XboxController.Button.kLeftBumper.value);
	}
}
