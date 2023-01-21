package frc.robot.oi;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Log;

public class DriverOI {
	private XboxController controller;

	public DriverOI(XboxController controller) {
		this.controller = controller;

		this.controller.a(new EventLoop());
		this.debugAddButtonLog(this.controller::b, "B");
		this.debugAddButtonLog(this.controller::x, "X");
		this.debugAddButtonLog(this.controller::y, "Y");
	}

	private void debugAddButtonLog(Consumer<EventLoop> fn, String name) {
		EventLoop loop = new EventLoop();
		loop.bind(() -> Log.writeln("Button press: " + name));
		fn.accept(loop);
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
}