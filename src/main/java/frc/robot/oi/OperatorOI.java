package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Log;

public class OperatorOI extends OIBase {
	public OperatorOI(XboxController controller) {
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
