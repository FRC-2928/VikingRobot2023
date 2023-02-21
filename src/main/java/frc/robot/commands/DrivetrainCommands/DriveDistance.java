package frc.robot.commands.DrivetrainCommands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Log;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
	private final Drivetrain drivetrain;
	private final double distance;
	private final double speed;
	private Pose2d endPose;

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
		Log.writeln("DriveDistance: " + this.distance);
		// Command will calculate the distance from zero
		// Any subsequent trajectory commands will reset this to
		// the start pose.
		// this.drivetrain.resetOdometry(new Pose2d());
		this.endPose = this.drivetrain.getEncoderPose()
			.plus(new Transform2d(new Translation2d(this.distance, 0), new Rotation2d()));
		Log.writeln("DriveDistance Start Pose: " + this.drivetrain.getEncoderPose());
	}

	@Override
	public void execute() {
		this.drivetrain.tankDriveVolts(speed * 12 * .95, speed * 12);
		// this.drivetrain.diffDrive.arcadeDrive(this.speed, 0);
	}

	@Override
	public void end(boolean interrupted) {
		this.drivetrain.halt();
		Log.writeln("DriveDistance End Pose: " + this.drivetrain.getEncoderPose());
	}

	@Override
	public boolean isFinished() {
		// return Math.abs(this.drivetrain.getAvgDistanceMeters()) >= this.distance;
		// TODO take care of negative distances
		return this.drivetrain.getEncoderPose().getX() >= endPose.getX();
	}
}
