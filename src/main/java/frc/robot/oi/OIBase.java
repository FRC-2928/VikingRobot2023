package frc.robot.oi;

import java.time.Duration;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public abstract class OIBase {
	public final XboxController controller;

    protected OIBase(XboxController controller) {
        this.controller = controller;
    }

    public void rumble(RumbleType rt, double intensity, Duration time) {
        this.controller.setRumble(rt, intensity);
        
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
            new WaitCommand(time.toSeconds()),
            new InstantCommand(() -> this.controller.setRumble(rt, 0))
        ));
    }

    public void signalError() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
            new InstantCommand(() -> this.controller.setRumble(RumbleType.kLeftRumble, 1)),
            new WaitCommand(0.25),
            new InstantCommand(() -> this.controller.setRumble(RumbleType.kLeftRumble, 0.5)),
            new WaitCommand(1.0),
            new InstantCommand(() -> this.controller.setRumble(RumbleType.kLeftRumble, 0))
        ));
    }
}
