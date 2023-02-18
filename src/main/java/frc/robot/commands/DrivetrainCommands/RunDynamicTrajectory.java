package frc.robot.commands.DrivetrainCommands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Log;

public class RunDynamicTrajectory extends RamseteCommand {
	private Drivetrain drivetrain;
	private Supplier<Trajectory> trajectory;
  private Pose2d initialPose;
	
	public RunDynamicTrajectory(Drivetrain drivetrain, Supplier<Trajectory> trajectory) {
		super(
			trajectory.get(),
			drivetrain::getEncoderPose,
			new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
			DrivetrainConstants.kDriveKinematics,
			drivetrain::setOutputMetersPerSecond,
			drivetrain
		);
		this.drivetrain = drivetrain;
		this.trajectory = trajectory;
		this.addRequirements(drivetrain);
	}

	public void initialize() {
		super.initialize();
    initialPose = this.trajectory.get().getInitialPose();
		Log.writeln("RunDynamicTrajectory Initial Pose: " + initialPose);
		this.drivetrain.resetOdometry(initialPose);
		this.drivetrain.disableMotorSafety();

		Log.writeln("Odometry Pose: " + this.drivetrain.getPose());
		SmartDashboard.putNumber("Y start traj", initialPose.getY());
		SmartDashboard.putNumber("Y start odom", this.drivetrain.getEncoderPose().getY());
		SmartDashboard.putNumber("X start traj", initialPose.getX());
		SmartDashboard.putNumber("X start odom", this.drivetrain.getEncoderPose().getX());
		SmartDashboard.putNumber("start odom heading", this.drivetrain.getEncoderPose().getRotation().getDegrees());
		SmartDashboard.putNumber("start traj heading", initialPose.getRotation().getDegrees());  

		// printTrajectory();
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
		this.drivetrain.enableMotorSafety();
	}

	public void printTrajectory() {
		Log.writeln("Initial Pose: " + initialPose);
		
		List<State> states = this.trajectory.get().getStates();
		for (int i = 1; i < states.size(); i++) {
		  var state = states.get(i);
		  Log.writeln("Time:" + state.timeSeconds + " Velocity:" + state.velocityMetersPerSecond);
		}  
	}
}
