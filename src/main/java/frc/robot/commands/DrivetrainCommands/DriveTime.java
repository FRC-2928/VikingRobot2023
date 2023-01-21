package frc.robot.commands.DrivetrainCommands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTime extends CommandBase {
	private final double duration;
	private final double speed;
	private final Drivetrain drivetrain;
	
	private long startTime;

	/**
	 * Creates a new DriveTime. This command will drive your robot for a desired speed and time.
	 *
	 * @param speed The speed which the robot will drive. Negative is in reverse.
	 * @param time How much time to drive in seconds
	 * @param drive The drivetrain subsystem on which this command will run
	 */
	public DriveTime(double speed, double time, Drivetrain drive) {
		this.speed = speed;
		this.duration = time * 1000;
		this.drivetrain = drive;
		this.addRequirements(drive);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		this.startTime = System.currentTimeMillis();
		this.drivetrain.halt();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		this.drivetrain.diffDrive.arcadeDrive(this.speed, 0);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		this.drivetrain.halt();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return (System.currentTimeMillis() - this.startTime) >= this.duration;
	}
}
