package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Log;

public class GenerateTrajectory extends InstantCommand {
	Drivetrain drivetrain;
	Pose2d endPose;
	int direction;

	public GenerateTrajectory(Drivetrain drivetrain, Pose2d endPose, int direction) {
		this.drivetrain = drivetrain;
		this.endPose = endPose;
		this.direction = direction;
		this.addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		Pose2d startPose = this.drivetrain.getEstimatedPose();
		Log.writeln("Initial Pose: " + startPose.getX());
		SmartDashboard.putNumber("Start Pose X", startPose.getX());
		SmartDashboard.putNumber("Start Pose Y", startPose.getY());
		SmartDashboard.putNumber("Start Pose Theta", startPose.getRotation().getDegrees());

		SmartDashboard.putNumber("End Pose X", this.endPose.getX());
		SmartDashboard.putNumber("End Pose Y", this.endPose.getY());
		SmartDashboard.putNumber("End Pose Theta", this.endPose.getRotation().getDegrees());

		// DriverStation.Alliance color = DriverStation.getAlliance();
		
		// if(color == DriverStation.Alliance.Red) {
		// 	// for red, left and right
		// 	//if direction is specified left, or direction is unspecified and Y is on left side of field...
		// 	if(direction == 0 || ((direction == 2 ) && (startPose.getY() <= (DrivetrainConstants.fieldWidthYMeters / 2)))) {
		// 		RobotContainer.dynamicTrajectory = TrajectoryGenerator.generateTrajectory(startPose, 
		// 		//List.of(DrivetrainConstants.leftRedWaypoint1, DrivetrainConstants.leftRedWaypoint2), \
		// 		List.of(),
		// 		endPose, DrivetrainConstants.kTrajectoryConfig);
		// 	} else {
		// 		RobotContainer.dynamicTrajectory = TrajectoryGenerator.generateTrajectory(startPose, 
		// 			//List.of(DrivetrainConstants.rightRedWaypoint1, DrivetrainConstants.rightRedWaypoint2), 
		// 			List.of(),
		// 			endPose, DrivetrainConstants.kTrajectoryConfig);
		// 	}
		// } else {
		// 	// for blue, left and right
		// 	if(direction == 0 || ((direction == 2 ) && (startPose.getY() >= (DrivetrainConstants.fieldWidthYMeters / 2)))) {
		// 		RobotContainer.dynamicTrajectory = TrajectoryGenerator.generateTrajectory(startPose, 
		// 			//List.of(DrivetrainConstants.leftBlueWaypoint1, DrivetrainConstants.leftBlueWaypoint2), 
		// 			List.of(),
		// 			endPose, DrivetrainConstants.kTrajectoryConfig);
		// 	} else {
		// 		RobotContainer.dynamicTrajectory = TrajectoryGenerator.generateTrajectory(startPose, 
		// 			//List.of(DrivetrainConstants.rightBlueWaypoint1, DrivetrainConstants.rightBlueWaypoint2), 
		// 			List.of(),
		// 			endPose, DrivetrainConstants.kTrajectoryConfig);
		// 	}
		// }

		SmartDashboard.putNumber("Waypoint1 X", FieldConstants.Waypoints.rightBlue1.getX());
		SmartDashboard.putNumber("Waypoint Y", FieldConstants.Waypoints.rightBlue1.getY());

		// RobotContainer.dynamicTrajectory = TrajectoryGenerator.generateTrajectory(startPose, 
		// 		List.of(FieldConstants.Waypoints.rightBlue1),
		// 		// List.of(),
		// 		endPose, DrivetrainConstants.kTrajectoryConfig);

	// Log.writeln("Traj: " + RobotContainer.dynamicTrajectory.getTotalTimeSeconds());    
	}
}
