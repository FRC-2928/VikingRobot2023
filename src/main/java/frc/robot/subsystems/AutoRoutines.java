// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.DrivetrainCommands.BalanceAUX;
import frc.robot.commands.DrivetrainCommands.BalancePID;
import frc.robot.commands.DrivetrainCommands.DriveDistance;
import frc.robot.commands.DrivetrainCommands.DriveTime;
import frc.robot.commands.DrivetrainCommands.RunRamseteTrajectory;
import frc.robot.commands.DrivetrainCommands.TurnToPid;
import frc.robot.subsystems.TrajectoryRunner.Direction;

/** Add your docs here. */
public class AutoRoutines {

    private final SendableChooser<Command> chooser = new SendableChooser<>();
    Drivetrain drivetrain;

    // Direction around the Charger Station
	// public static enum Direction {
	// 	Left,
	// 	Right,
	// 	Center
	// }

    public AutoRoutines(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public SendableChooser<Command> configureAutoChooser() {
		

		chooser.addOption("Tag7/2 Balance",
			new SequentialCommandGroup(
				//new DriveTime(-.6, 4.5, drivetrain),
				new BalancePID(drivetrain, false, 10),BalanceAUX.manual(drivetrain)
			)
		);
		
		chooser.addOption(
			"Tag1 Routines1", 
			new SequentialCommandGroup( 
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Tag6-Rotate3.0")),
				new TurnToPid(180, drivetrain),
				new DriveDistance(.5, .5, drivetrain),
				new TurnToPid(0, drivetrain),
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Rotate3.0-Tag6")),
				TrajectoryRunner.generateRamseteCommand(drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(drivetrain, Direction.Center))
			)
		);

		chooser.addOption("Tag3 Routines1",
			new SequentialCommandGroup(
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Tag8-Rotate4")),
				new TurnToPid(180, drivetrain),
				new DriveDistance(.5, .5, drivetrain),
				new TurnToPid(0, drivetrain),
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Rotate4-AprilTag8")),
				TrajectoryRunner.generateRamseteCommand(drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(drivetrain, Direction.Center))
			)
		);		

		chooser.addOption(
			"Tag6 Routines1", 
			new SequentialCommandGroup( 
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Tag6-Rotate3.0")),
				new TurnToPid(0, drivetrain),
				new DriveDistance(.5, .5, drivetrain),
				new TurnToPid(180, drivetrain),
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Rotate3.0-Tag6")),
				TrajectoryRunner.generateRamseteCommand(drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(drivetrain, Direction.Center))
			)
		);

		chooser.addOption("Tag8 Routines1",
			new SequentialCommandGroup(
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Tag8-Rotate4")),
				new TurnToPid(0, drivetrain),
				new DriveDistance(.5, .5, drivetrain),
				new TurnToPid(180, drivetrain),
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Rotate4-AprilTag8")),
				TrajectoryRunner.generateRamseteCommand(drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(drivetrain, Direction.Center))
			)
		);

		chooser.addOption("Tag8/3 Balance", 
			new SequentialCommandGroup(
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Tag8-Around-Balance")),
				new BalancePID(drivetrain, false, 10),BalanceAUX.manual(drivetrain)
			)	
		);

		chooser.addOption("Tag6/1 Balance", 
			new SequentialCommandGroup(
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Tag6-Around-Balance")),
				new BalancePID(drivetrain, false, 10),BalanceAUX.manual(drivetrain)
			)	
		); 

		chooser.addOption("Calibrate Trajectory", 
			new RunRamseteTrajectory(drivetrain, calibrateTrajectory()));

        return chooser; 
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
  	// private Command generateRamseteCommand(Supplier<Trajectory> trajectory) {
	// 	Log.writeln("generateramsetecommand");
	// 	if (trajectory.get() == null) {
	// 		Log.writeln("generateRamseteCommand: Got null trajectory!");
	// 		return new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0), drivetrain);
	// 	}

	// 	RamseteCommand ramseteCommand = new RamseteCommand(
	// 		trajectory.get(),
	// 		drivetrain::getPose,
	// 		new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
	// 		DrivetrainConstants.kDriveKinematics,
	// 		drivetrain::setOutputMetersPerSecond,
	// 		drivetrain);

	// 	// drivetrain.resetOdometry(trajectory.get().getInitialPose());

	// 	// Set up a sequence of commands
	// 	// First, we want to reset the drivetrain odometry
	// 	return new InstantCommand(() -> {
	// 		try {
	// 			Field field = ramseteCommand.getClass().getDeclaredField("m_trajectory");
	// 			field.setAccessible(true);
	// 			field.set(ramseteCommand, trajectory.get());
	// 			Log.writeln("field", field, trajectory.get());
	// 			Log.writeln("fields", ramseteCommand.getClass().getFields());
	// 			this.drivetrain.resetOdometry(trajectory.get().getInitialPose());
	// 		} catch(Exception e) {
	// 			Log.error(e);
	// 		}
	// 	}, this.drivetrain)
	// 		// next, we run the actual ramsete command
	// 		.andThen(ramseteCommand)
	// 		// make sure that the robot stops
	// 		.andThen(new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0), drivetrain));
	// } 

    // public Trajectory generateLocalTrajectory(Direction direction) {
	// 	Log.writeln("generateLocalTrajectory");
	// 	if (this.drivetrain.hasNoLimelightTarget()) {
	// 		Log.writeln("generateLocalTrajectory: No Limelight target!");
	// 		return null;
	// 	}

	// 	// Get the aprilTag that the robot is looking at and it's pose relative to the tag.
	// 	Pose2d startPose = this.drivetrain.getLimelightPoseRelative();
	// 	double aprilTagID = this.drivetrain.getAprilTagID();
	// 	if(!FieldConstants.aprilTags.containsKey((int)aprilTagID)){
	// 		Log.writeln("generateRamseteCommand: No valid aprilTag!" + aprilTagID);
	// 		return null;
	// 	}

	// 	// Now get the pose
	// 	Pose2d tag = FieldConstants.aprilTags.get((int)aprilTagID).toPose2d();
	// 	Pose2d endPose;
	// 	switch(direction){
	// 		case Left:
	// 		// endPose = tag.plus(new Transform2d(new Translation2d(0.75, -Units.inchesToMeters(22.5)), new Rotation2d(Math.PI)));
	// 		endPose = tag.plus(FieldConstants.leftOffset);	
	// 		break;
	// 		case Right:
	// 		// endPose = tag.plus(new Transform2d(new Translation2d(0.75, Units.inchesToMeters(22.5)), new Rotation2d(Math.PI)));	
	// 		endPose = tag.plus(FieldConstants.rightOffset);	
	// 		break;
	// 		default:
	// 		// endPose = tag.plus(new Transform2d(new Translation2d(0.75, 0), new Rotation2d(Math.PI)));
	// 		endPose = tag.plus(FieldConstants.centerOffset);		
	// 		break;		
	// 	}

	// 	SmartDashboard.putNumber("Start Pose X", startPose.getX());
    //     SmartDashboard.putNumber("Start Pose Y", startPose.getY());
    //     SmartDashboard.putNumber("Start Pose Heading", startPose.getRotation().getDegrees());
    
    //     SmartDashboard.putNumber("End Pose X", endPose.getX());
    //     SmartDashboard.putNumber("End Pose Y", endPose.getY());
    //     SmartDashboard.putNumber("End Pose Heading", endPose.getRotation().getDegrees());
        
	// 	Trajectory trajectory;
	// 	List<Translation2d> waypoints = new ArrayList<>();

	// 	// waypoints.add(new Translation2d(endPose.getX() + 1, endPose.getY() + 0.1));
	// 	trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
	// 				waypoints,
    //     			endPose, AutoConstants.kTrajectoryConfig);

	// 	Log.writeln("Initial Pose: " + trajectory.getInitialPose());
	// 	Log.writeln("Waypoints:" + waypoints);
	// 	Log.writeln("End Pose:" + endPose);

    //     this.printTrajectory(trajectory);

    //     return trajectory;

	// }

	public Trajectory loadTrajectory(String trajectoryJSON) {
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

    /**
	 * Drives a straight line 4 meters so as you can calibrate your Romi
	 * You should make sure that the robot ends up right on the 2 meter mark.
	 *
	 */
  	public Trajectory calibrateTrajectory() {
		// Note that all coordinates are in meters, and follow NWU conventions.
		return TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// List.of(new Translation2d(2.0, 0.0)) 
			List.of(),
			new Pose2d(4, 0.0, new Rotation2d(0)), // left
			AutoConstants.kTrajectoryConfig);
  	}

	// public void printTrajectory(Trajectory trajectory) {
	// 	List<State> states = trajectory.getStates();

	// 	for (int i = 1; i < states.size(); i++) {
	// 		var state = states.get(i);
	// 		Log.writeln("Time:" + state.timeSeconds + 
	// 					" X:" + state.poseMeters.getX() +
	// 					" Y:" + state.poseMeters.getY() +
	// 					" Vel:" + state.velocityMetersPerSecond +
	// 					" Curvature:" + state.curvatureRadPerMeter);
	// 	}  

	// 	Log.writeln("Traj: " + trajectory.getTotalTimeSeconds()); 
	// }	

}
