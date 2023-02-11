package frc.robot.commands.DrivetrainCommands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
	private final Drivetrain drivetrain;
	private final double distance;
	private final double speed;

	/**
	 * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
	 * a desired speed.
	 *
	 * @param speed The speed at which the robot will drive
	 * @param meters The number of meters the robot will drive
	 * @param drivetrain The drivetrain subsystem on which this command will run
	 */
	public DriveDistance(double speed, double meters, Drivetrain drivetrain) {
		this.distance = meters;
		this.speed = speed;
		this.drivetrain = drivetrain;
		this.addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		this.drivetrain.halt();
		this.drivetrain.resetEncoders(); // todo: is this a good idea?
	}

	@Override
	public void execute() {
		this.drivetrain.tankDriveVolts(speed * 12 * .95, speed * 12);
		// this.drivetrain.diffDrive.arcadeDrive(this.speed, 0);
	}

	@Override
	public void end(boolean interrupted) {
		this.drivetrain.halt();
	}

	@Override
	public boolean isFinished() {
		return Math.abs(this.drivetrain.getAvgDistanceMeters()) >= this.distance;
	}
}
