package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Log;

public class DriverOI extends OIBase {
	/*

	A - elevator down to start match
	B - 
	X - 
	Y - balance

	Start(right) - reset gyro
	Back(left) - halt

	LB - start pov selector
	RB - runs the intake

	LT balance aux
	RT

	LS - y-move
	RS - x-rotate

	LS Click - shift high
	RS Click - shift low

	*/
	
	public DriverOI(XboxController controller) {
		super(controller);

		this.debugAddButtonLog(XboxController.Button.kA.value, "A");
		this.debugAddButtonLog(XboxController.Button.kB.value, "B");
		this.debugAddButtonLog(XboxController.Button.kX.value, "X");
		this.debugAddButtonLog(XboxController.Button.kY.value, "Y");
		this.debugAddButtonLog(XboxController.Button.kStart.value, "Start");
		this.debugAddButtonLog(XboxController.Button.kBack.value, "Back");
		this.debugAddButtonLog(XboxController.Button.kLeftBumper.value, "Left Bumper");
		this.debugAddButtonLog(XboxController.Button.kRightBumper.value, "Right Bumper");
		this.debugAddButtonLog(XboxController.Button.kLeftStick.value, "Left Stick");
		this.debugAddButtonLog(XboxController.Button.kRightStick.value, "Right Stick");
	}

	private void debugAddButtonLog(int id, String message) {
		new JoystickButton(this.controller, id)
			.onTrue(new InstantCommand(() -> Log.writeln("Pressed " + message)))
			.onFalse(new InstantCommand(() -> Log.writeln("Released " + message)));
	}

	// ---------------- Drivetrain ----------------------------

	public DoubleSupplier getMoveSupplier() {
		return () -> -this.controller.getLeftY();
	}

	// public DoubleSupplier getMoveRSupplier() {
	// 	return () -> -this.controller.getRightY();
	// }

	public DoubleSupplier getRotateSupplier() {
		return () -> this.controller.getRightX();
	}

	// public Trigger getMoveToPlaceHigh() {
	// 	return new JoystickButton(this.controller, XboxController.Button.kB.value);
	// }

	// public Trigger getMoveToPlaceMid() {
	// 	return new JoystickButton(this.controller, XboxController.Button.kA.value);
	// }

	

	

	// Shifting

	public Trigger getShiftLowButton() {
		return new JoystickButton(this.controller, XboxController.Button.kRightStick.value);
	}

	public Trigger getShiftHighButton() {
		return new JoystickButton(this.controller, XboxController.Button.kLeftStick.value);
	}

	// public Trigger getCoastBrakeButton() {
	// 	return new JoystickButton(this.controller, XboxController.Button.kX.value);
	// }

	// Balance

	// public Trigger getBalanceButton() {
	// 	return new JoystickButton(this.controller, XboxController.Button.kRightBumper.value);
	// }

	// public Trigger getRollButton() {
	// 	return new JoystickButton(this.controller, XboxController.Button.kLeftBumper.value);
	// }

	public Trigger getBalanceAuxButton() {
		return new JoystickButton(this.controller, XboxController.Button.kY.value);
	}

	// Misc

	public Trigger getApproachTagButton() {
		return new JoystickButton(this.controller, XboxController.Button.kLeftBumper.value);
	}

	public Trigger getResetGyroButton() {
		return new JoystickButton(this.controller, XboxController.Button.kStart.value);
	}

	public Trigger getRunIntakeButton() {
		return new JoystickButton(this.controller, XboxController.Button.kRightBumper.value);
	}

	public Trigger getOrchestraButton() {
		return new JoystickButton(this.controller, XboxController.Button.kLeftStick.value);
	}

	public Trigger getElevatorToStartButton(){
		return new JoystickButton(this.controller, XboxController.Button.kA.value);
	}
}