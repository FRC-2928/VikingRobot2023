package frc.robot.oi;

import java.time.Duration;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

public abstract class OIBase {
	public final XboxController controller;

	protected OIBase(XboxController controller) {
		this.controller = controller;

		controller.setRumble(RumbleType.kBothRumble, 0);
	}

	public final void rumble(RumbleType rt, double intensity, Duration time) {
		this.controller.setRumble(rt, intensity);

		CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
			new WaitCommand(time.toSeconds()),
			new InstantCommand(() -> this.controller.setRumble(rt, 0))
		));
	}

	public final void signalError() {
		CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
			new InstantCommand(() -> this.controller.setRumble(RumbleType.kLeftRumble, 1)),
			new WaitCommand(0.25),
			new InstantCommand(() -> this.controller.setRumble(RumbleType.kLeftRumble, 0.5)),
			new WaitCommand(1.0),
			new InstantCommand(() -> this.controller.setRumble(RumbleType.kLeftRumble, 0))
		));
	}

	public Trigger getHaltButton() {
		return new JoystickButton(this.controller, XboxController.Button.kBack.value);
	}
}
