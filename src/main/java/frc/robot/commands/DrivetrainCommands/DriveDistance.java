package frc.robot.commands.DrivetrainCommands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
	private final Drivetrain drivetrain;
	private final double distance;
	private final double speed;
	private Pose2d endPose;

	public DriveDistance(double speed, double meters, Drivetrain drivetrain) {
		this.distance = -meters;
		this.speed = -speed;
		this.drivetrain = drivetrain;
		this.addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		this.drivetrain.halt();
		// Command will calculate the distance from zero
		// Any subsequent trajectory commands will reset this to
		// the start pose.
		// this.drivetrain.resetOdometry(new Pose2d());
		this.endPose = this.drivetrain
			.getEncoderPose()
			.plus(new Transform2d(new Translation2d(-this.distance, 0), new Rotation2d()));
	}

	@Override
	public void execute() {
		this.drivetrain.tankDriveVolts(-this.speed * 12, -this.speed * 12);
	}

	@Override
	public void end(boolean interrupted) {
		this.drivetrain.halt();
	}

	@Override
	public boolean isFinished() {
		if(this.distance >= 0) {
			return this.drivetrain.getEncoderPose().getX() <= this.endPose.getX();
		} else {
			return this.drivetrain.getEncoderPose().getX() >= this.endPose.getX();
		}
	}
}
