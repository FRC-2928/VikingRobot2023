package frc.robot.commands.DrivetrainCommands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnTime extends CommandBase {
	private final double duration;
	private final double rotationalSpeed;
	private final Drivetrain drivetrain;

	private long startTime;

	/**
	 * Creates a new TurnTime.
	 *
	 * @param speed The speed which the robot will turn. Negative is in reverse.
	 * @param time How much time to turn in seconds
	 * @param drive The drive subsystem on which this command will run
	 */
	public TurnTime(double speed, double time, Drivetrain drive) {
		this.rotationalSpeed = speed;
		this.duration = time * 1000;
		this.drivetrain = drive;
		this.addRequirements(drive);
	}

	@Override
	public void initialize() {
		this.startTime = System.currentTimeMillis();
		this.drivetrain.halt();
	}

	@Override
	public void execute() {
		this.drivetrain.diffDrive.arcadeDrive(0, this.rotationalSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		this.drivetrain.halt();
	}

	@Override
	public boolean isFinished() {
		return (System.currentTimeMillis() - this.startTime) >= this.duration;
	}
}
