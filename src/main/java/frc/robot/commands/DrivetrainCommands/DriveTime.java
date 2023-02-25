package frc.robot.commands.DrivetrainCommands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTime extends CommandBase {
	private final double duration;
	private final double speed;
	private final Drivetrain drivetrain;

	private long startTime;

	public DriveTime(double speed, double time, Drivetrain drive) {
		this.speed = speed;
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
		this.drivetrain.diffDrive.arcadeDrive(this.speed, 0);
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
