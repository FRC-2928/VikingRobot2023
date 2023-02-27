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

	LS up and down controls elevator, right and left controls arm
		(probably most movement for these subsystems will be to predesignated positions)
	RS

	LS Click
	RS Click

	potentially POV for elevator/arm positions - high, middle, ground
		- thinking on way up move elevator and then arm, on way down move arm and then elevator
		- sequential commands to do one and then the other

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

	public Trigger getHigh() {
		return new Trigger(() -> this.controller.getPOV() == 0);
	}

	public Trigger getMid() {
		return new Trigger(() -> this.controller.getPOV() == 90);
	}

	public Trigger getLow() {
		return new Trigger(() -> this.controller.getPOV() == 180);
	}
}
