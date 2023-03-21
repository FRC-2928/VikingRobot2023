package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Transmission;
import frc.robot.subsystems.Transmission.GearState;

public class Shift extends CommandBase {
	private final Transmission transmission;
	private final GearState newState;
	private GearState previousState;

	public Shift(Transmission trans, GearState to) {
		this.transmission = trans;
		this.newState = to;

		this.addRequirements(trans);
	}

	@Override
	public void initialize() {
		this.previousState = this.transmission.getGearState();
		this.transmission.shift(this.newState);
	}

	@Override
	public void end(boolean interrupted) {
		this.transmission.shift(this.previousState);
	}
}
