package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Log;

public class DriverOI extends OIBase {
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
			.onTrue(new InstantCommand(() -> Log.writeln("press " + message)))
			.onFalse(new InstantCommand(() -> Log.writeln("release " + message)));
	}

	// ---------------- Drivetrain ----------------------------

	public DoubleSupplier getRotateSupplier() {
		return () -> this.controller.getRightX();
	}

	public Trigger getShiftLowButton() {
		return new JoystickButton(this.controller, XboxController.Button.kX.value);
	}

	public Trigger getShiftHighButton() {
		return new JoystickButton(this.controller, XboxController.Button.kY.value);
	}

	public Trigger getOrchestraButton() {
		return new JoystickButton(this.controller, XboxController.Button.kStart.value);
	}

	public DoubleSupplier getMoveSupplier() {
		return () -> -this.controller.getLeftY();
	}

	public Trigger getIsAtHighSpeed() {
		return new Trigger(() -> Math.abs(this.controller.getLeftY()) > .85);
	}

	public Trigger getBalanceButton() {
		return new JoystickButton(this.controller, XboxController.Button.kRightBumper.value);
	}

	public Trigger getRollButton() {
		return new JoystickButton(this.controller, XboxController.Button.kLeftBumper.value);
	}

	public Trigger getResetGyroButton() {
		return new JoystickButton(this.controller, XboxController.Button.kB.value);
	}

	public Trigger getBalanceAuxButton() {
		return new JoystickButton(this.controller, XboxController.Button.kA.value);
	}

	public Trigger getTestButton() {
		return new JoystickButton(this.controller, XboxController.Button.kBack.value);
	}

	public Trigger getStartButton() {
		return new JoystickButton(this.controller, XboxController.Button.kStart.value);
	}
	
	public Trigger getGoToTag6Button(){
		return new Trigger(() -> this.controller.getPOV() == 90);
	}

	public Trigger getGoToTag7Button(){
		return new Trigger(() -> this.controller.getPOV() == 180);
	}

	public Trigger getGoToTag8Button(){
		return new Trigger(() -> this.controller.getPOV() == 270);
	}

}