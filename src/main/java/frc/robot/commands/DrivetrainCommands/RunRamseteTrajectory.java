package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class RunRamseteTrajectory extends RamseteCommand {
	private Drivetrain drivetrain;
	private Trajectory trajectory;
	
	public RunRamseteTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
		super(
			trajectory,
			drivetrain::getEncoderPose,
			new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
			DrivetrainConstants.kDriveKinematics,
			drivetrain::setOutputMetersPerSecond,
			drivetrain
		);
		this.drivetrain = drivetrain;
		this.trajectory = trajectory;
		addRequirements(drivetrain);
	}

	public void initialize() {
		super.initialize();
		this.drivetrain.resetOdometry(this.trajectory.getInitialPose());
		System.out.println("Initial Pose: " + this.trajectory.getInitialPose());
		SmartDashboard.putNumber("Y start traj", this.trajectory.getInitialPose().getY());
		SmartDashboard.putNumber("Y start odom", this.drivetrain.getEncoderPose().getY());
		SmartDashboard.putNumber("X start traj", this.trajectory.getInitialPose().getX());
		SmartDashboard.putNumber("X start odom", this.drivetrain.getEncoderPose().getX());
		SmartDashboard.putNumber("start odom heading", this.drivetrain.getEncoderPose().getRotation().getDegrees());
		SmartDashboard.putNumber("start traj heading", this.trajectory.getInitialPose().getRotation().getDegrees());  
	}

	public void execute() {
		super.execute();
		SmartDashboard.putNumber("Odometry X", this.drivetrain.getEncoderPose().getX());
		SmartDashboard.putNumber("Odometry Y", this.drivetrain.getEncoderPose().getY());
		SmartDashboard.putNumber("Odometry heading", this.drivetrain.getEncoderPose().getRotation().getDegrees());
		//SmartDashboard.putNumber("current heading", this.drivetrain.getHeading());
		this.drivetrain.diffDrive.feed(); // Feed motor safety instead of disabling it
	}

	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);
		//SmartDashboard.putNumber("end heading", this.drivetrain.getHeading());
		this.drivetrain.halt();
	}

}
