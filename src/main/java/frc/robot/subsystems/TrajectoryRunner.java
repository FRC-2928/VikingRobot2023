// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;

/** Add your docs here. */
public class TrajectoryRunner {
    public static enum Direction {
		Left,
		Right,
		Center
	}
    
    /**
	 * Generates a dynamic trajectory starting at the current pose of the
	 * robot, as determined by the Limelight looking at the AprilTags.
	 * The trajectory will end at the current in-view apriltag, or to the
	 * left or right of it as directed.
	 * 
	 * @param direction whether the robot should end at the center, left,
	 *                  or right of the apriltag.
	 * 
	 * @return The generated Trajectory object
	 */
	public static Trajectory generateLocalTrajectory(Drivetrain drivetrain, Direction direction) {
		Log.writeln("generateLocalTrajectory");
		Trajectory trajectory = new Trajectory();
		
		// Get the aprilTag that the robot is looking at
		double aprilTagID = drivetrain.getAprilTagID();

		// Default trajectory if no limelight target is to move back 0.5 meters
		Pose2d startPose = drivetrain.getEncoderPose();
		Pose2d endPose = startPose.plus(new Transform2d(new Translation2d(-0.5, 0), 
										new Rotation2d()));
		// Set the trajectory config to reversed.
		TrajectoryConfig config = AutoConstants.kTrajectoryConfigReversed;
		
		List<Translation2d> waypoints = new ArrayList<>();
	
		if (drivetrain.hasNoLimelightTarget()) {			
			Log.writeln("No Limelight target!");	
			Log.writeln("Start Pose X ", startPose.getX());
			Log.writeln("Start Pose Y ", startPose.getY());
			Log.writeln("Start Pose Heading ", startPose.getRotation().getDegrees());
		
			Log.writeln("End Pose X ", endPose.getX());
			Log.writeln("End Pose Y ", endPose.getY());
			Log.writeln("End Pose Heading ", endPose.getRotation().getDegrees());
			
			Robot.instance.robotContainer.driverOI.signalError();

			return new Trajectory();
		} else if(!FieldConstants.aprilTags.containsKey((int)aprilTagID)) {
			Log.writeln("Invalid aprilTag! " + aprilTagID);	

		} else {
			// Get the aprilTag that the robot is looking at and it's pose relative to the tag.
			startPose = drivetrain.getLimelightPoseRelative();
			// Move forward config
			config = AutoConstants.kTrajectoryConfig;

			// Now get the pose
			Pose2d tag = FieldConstants.aprilTags.get((int)aprilTagID).toPose2d();
			switch(direction) {
				case Left:
				// endPose = tag.plus(new Transform2d(new Translation2d(0.75, -Units.inchesToMeters(22.5)), new Rotation2d(Math.PI)));
				endPose = tag.plus(FieldConstants.leftOffset);	
				break;
				case Right:
				// endPose = tag.plus(new Transform2d(new Translation2d(0.75, Units.inchesToMeters(22.5)), new Rotation2d(Math.PI)));	
				endPose = tag.plus(FieldConstants.rightOffset);	
				break;
				default:
				// endPose = tag.plus(new Transform2d(new Translation2d(0.75, 0), new Rotation2d(Math.PI)));
				endPose = tag.plus(FieldConstants.centerOffset);		
				break;		
			}
	
		}

		SmartDashboard.putNumber("Start Pose X", startPose.getX());
        SmartDashboard.putNumber("Start Pose Y", startPose.getY());
        SmartDashboard.putNumber("Start Pose Heading", startPose.getRotation().getDegrees());
    
        SmartDashboard.putNumber("End Pose X", endPose.getX());
        SmartDashboard.putNumber("End Pose Y", endPose.getY());
        SmartDashboard.putNumber("End Pose Heading", endPose.getRotation().getDegrees());
        
		// waypoints.add(new Translation2d(endPose.getX() + 1, endPose.getY() + 0.1));
		trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
					waypoints,
        			endPose, config);

		Log.writeln("Initial Pose: " + trajectory.getInitialPose());
		Log.writeln("Waypoints:" + waypoints);
		Log.writeln("End Pose:" + endPose);

        printTrajectory(trajectory);

        return trajectory;
	}

    /**
	 * Generate a trajectory following Ramsete command
	 * 
	 * This is very similar to the WPILib RamseteCommand example. It uses
	 * constants defined in the Constants.java file. These constants were 
	 * found empirically by using the frc-characterization tool.
	 * 
	 * @return A SequentialCommand that sets up and executes a trajectory following Ramsete command
	 */
  	public static Command generateRamseteCommand(Drivetrain drivetrain, Supplier<Trajectory> trajectory) {
		RamseteCommand ramseteCommand = new RamseteCommand(
			new Trajectory(),
			drivetrain::getPose,
			new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
			DrivetrainConstants.kDriveKinematics,
			drivetrain::setOutputMetersPerSecond,
			drivetrain);

		// drivetrain.resetOdometry(trajectory.get().getInitialPose());

		// Set up a sequence of commands
		// First, we want to reset the drivetrain odometry
		return new InstantCommand(() -> {
			try {
				Trajectory traj = trajectory.get();
				Field field = ramseteCommand.getClass().getDeclaredField("m_trajectory");
				field.setAccessible(true);
				field.set(ramseteCommand, traj);
				drivetrain.resetOdometry(traj.getInitialPose());
			} catch(Exception e) {
				Log.error(e);
			}
		}, drivetrain)
			// next, we run the actual ramsete command
			.andThen(ramseteCommand)
			// make sure that the robot stops
			.andThen(new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0), drivetrain));
	} 
    
    /**
	 * Generates a dynamic trajectory starting at the current pose of the
	 * robot, as determined by the Limelight looking at the AprilTags.
	 * Additional waypoints may be added to navigate around field structures.
	 * 
	 * @param endPose pose where robot should end
	 * 
	 * @return The generated Trajectory object
	 */
	public static Trajectory generateTrajectory(Drivetrain drivetrain, Pose2d endPose) {
        Pose2d startPose = drivetrain.getLimelightPoseRelative();

        Log.writeln("Generate start Pose: " + startPose);
        SmartDashboard.putNumber("Start Pose X", startPose.getX());
        SmartDashboard.putNumber("Start Pose Y", startPose.getY());
        SmartDashboard.putNumber("Start Pose Heading", startPose.getRotation().getDegrees());
    
        SmartDashboard.putNumber("End Pose X", endPose.getX());
        SmartDashboard.putNumber("End Pose Y", endPose.getY());
        SmartDashboard.putNumber("End Pose Heading", endPose.getRotation().getDegrees());
        
		Trajectory trajectory;
		List<Translation2d> waypoints = new ArrayList<>();

		// Log.writeln("Alliance:" + alliance);
        // if(alliance == DriverStation.Alliance.Red) {
        // 	// for red, left and right
        // 	//if direction is specified left, or direction is unspecified and Y is on left side of field...
        // 	if(direction == Direction.Left || ((direction == Direction.Unspecified ) && (drivetrain.isLeftOfChargingStation()))) {
		// 		Log.writeln("Red left");
        // 		trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
		// 			List.of(FieldConstants.Waypoints.leftRed1, FieldConstants.Waypoints.leftRed2),
		// 			endPose, DrivetrainConstants.kTrajectoryConfig);
        // 	} else {
		// 		Log.writeln("Red right");
        // 		trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
		// 			List.of(FieldConstants.Waypoints.rightRed1, FieldConstants.Waypoints.rightRed2), 
		// 			endPose, DrivetrainConstants.kTrajectoryConfig);
        // 	}
        // } else {
        // 	// for blue, left and right
        // 	if(direction == Direction.Left || ((direction == Direction.Unspecified) && (drivetrain.isLeftOfChargingStation()))) {
        // 		Log.writeln("Blue left");
		// 		Log.writeln("CS Center" + FieldConstants.Community.chargingStationCenterY);
		// 		if (startPose.getX() > FieldConstants.Waypoints.leftBlue1.getX()) {
		// 			waypoints.add(FieldConstants.Waypoints.leftBlue1);
		// 		}
		// 		if (startPose.getX() > FieldConstants.Waypoints.leftBlue2.getX()) {
		// 			waypoints.add(FieldConstants.Waypoints.leftBlue2);
		// 		}				

		// 		trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
		// 			waypoints,
        // 			endPose, DrivetrainConstants.kTrajectoryConfig);
        // 	} else {
		// 		Log.writeln("Blue right");
		// 		if (startPose.getX() > FieldConstants.Waypoints.rightBlue1.getX()) {
		// 			waypoints.add(FieldConstants.Waypoints.rightBlue1);
		// 		}		
		// 		if (startPose.getX() > FieldConstants.Waypoints.rightBlue2.getX()) {
		// 			waypoints.add(FieldConstants.Waypoints.rightBlue2);
		// 		} 		
				
        // 		trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
		// 			waypoints,
        // 			endPose, DrivetrainConstants.kTrajectoryConfig);
        // 	}
        // }

    	// waypoints.add(new Translation2d(endPose.getX() + 2, endPose.getY() + 0.1));
		trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
					waypoints,
        			endPose, AutoConstants.kTrajectoryConfig);

		Log.writeln("Initial Pose: " + trajectory.getInitialPose());
		Log.writeln("Waypoints:" + waypoints);
		Log.writeln("End Pose:" + endPose);

        printTrajectory(trajectory);

        return trajectory;
    }

    public static Trajectory loadTrajectory(String trajectoryJSON) {
		Path trajectoryPath = Filesystem
			.getDeployDirectory()
			.toPath()
			.resolve("paths/output/" + trajectoryJSON + ".wpilib.json");
		
		try {
			return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			Log.error(ex);
			return null;
		}
	}

    public static void printTrajectory(Trajectory trajectory) {
		List<State> states = trajectory.getStates();

		for (int i = 1; i < states.size(); i++) {
			var state = states.get(i);
			Log.writeln("Time:" + state.timeSeconds + 
						" X:" + state.poseMeters.getX() +
						" Y:" + state.poseMeters.getY() +
						" Vel:" + state.velocityMetersPerSecond +
						" Curvature:" + state.curvatureRadPerMeter);
		}  

		Log.writeln("Traj: " + trajectory.getTotalTimeSeconds()); 
	}	
}
