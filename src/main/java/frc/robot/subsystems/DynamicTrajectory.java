// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.Constants.DrivetrainConstants;

/** Add your docs here. */
public class DynamicTrajectory {
    Drivetrain drivetrain;

    public DynamicTrajectory(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public Trajectory generateTrajectory(Pose2d endPose) {

        Pose2d startPose = this.drivetrain.getEstimatedPose();
        System.out.println("Initial Pose: " + startPose.getX());
        SmartDashboard.putNumber("Start Pose X", startPose.getX());
        SmartDashboard.putNumber("Start Pose Y", startPose.getY());
        SmartDashboard.putNumber("Start Pose Theta", startPose.getRotation().getDegrees());
    
        SmartDashboard.putNumber("End Pose X", endPose.getX());
        SmartDashboard.putNumber("End Pose Y", endPose.getY());
        SmartDashboard.putNumber("End Pose Theta", endPose.getRotation().getDegrees());
    
        DriverStation.Alliance color = DriverStation.getAlliance();
        
        // if(color == DriverStation.Alliance.Red){
        // 	// for red, left and right
        // 	//if direction is specified left, or direction is unspecified and Y is on left side of field...
        // 	if(direction == 0 || ((direction == 2 ) && (startPose.getY() <= (DrivetrainConstants.fieldWidthYMeters / 2)))){
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
        // 	if(direction == 0 || ((direction == 2 ) && (startPose.getY() >= (DrivetrainConstants.fieldWidthYMeters / 2)))){
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
    
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
                List.of(FieldConstants.Waypoints.rightBlue1),
                // List.of(),
                endPose, DrivetrainConstants.kTrajectoryConfig);
    
        System.out.println("Traj: " + trajectory.getTotalTimeSeconds()); 

        return trajectory;
    }
}


