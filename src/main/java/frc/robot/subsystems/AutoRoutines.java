// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DrivetrainCommands.Balance;
import frc.robot.commands.DrivetrainCommands.DriveDistance;
import frc.robot.commands.DrivetrainCommands.RunRamseteTrajectory;
import frc.robot.commands.DrivetrainCommands.TurnToPid;
import frc.robot.subsystems.TrajectoryRunner.Direction;

/** Add your docs here. */
public class AutoRoutines {

    private final SendableChooser<Command> chooser = new SendableChooser<>();
    Drivetrain drivetrain;

    public AutoRoutines(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public SendableChooser<Command> configureAutoChooser() {
		

		chooser.addOption("Tag7/2 Balance",
			new SequentialCommandGroup(
				//new DriveTime(-.6, 4.5, drivetrain),
				Balance.timed(drivetrain, 1000),Balance.manual(drivetrain)
			)
		);
		
		chooser.addOption(
			"Tag1 Routines1", 
			new SequentialCommandGroup( 
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Tag6-Rotate3.0")),
				new TurnToPid(180, drivetrain),
				new DriveDistance(.5, .5, drivetrain),
				new TurnToPid(0, drivetrain),
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Rotate3.0-Tag6")),
				TrajectoryRunner.generateRamseteCommand(drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(drivetrain, Direction.Center))
			)
		);

		chooser.addOption("Tag3 Routines1",
			new SequentialCommandGroup(
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Tag8-Rotate4")),
				new TurnToPid(180, drivetrain),
				new DriveDistance(.5, .5, drivetrain),
				new TurnToPid(0, drivetrain),
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Rotate4-AprilTag8")),
				TrajectoryRunner.generateRamseteCommand(drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(drivetrain, Direction.Center))
			)
		);		

		chooser.addOption(
			"Tag6 Routines1", 
			new SequentialCommandGroup( 
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Tag6-Rotate3.0")),
				new TurnToPid(0, drivetrain),
				new DriveDistance(.5, .5, drivetrain),
				new TurnToPid(180, drivetrain),
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Rotate3.0-Tag6")),
				TrajectoryRunner.generateRamseteCommand(drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(drivetrain, Direction.Center))
			)
		);

		chooser.addOption("Tag8 Routines1",
			new SequentialCommandGroup(
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Tag8-Rotate4")),
				new TurnToPid(0, drivetrain),
				new DriveDistance(.5, .5, drivetrain),
				new TurnToPid(180, drivetrain),
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Rotate4-AprilTag8")),
				TrajectoryRunner.generateRamseteCommand(drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(drivetrain, Direction.Center))
			)
		);

		chooser.addOption("Tag8/3 Balance", 
			new SequentialCommandGroup(
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Tag8-Around-Balance")),
				Balance.timed(drivetrain, 1000),Balance.manual(drivetrain)
			)	
		);

		chooser.addOption("Tag6/1 Balance", 
			new SequentialCommandGroup(
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Tag6-Around-Balance")),
				Balance.timed(drivetrain, 1000),Balance.manual(drivetrain)
			)	
		); 

		chooser.addOption("Calibrate Trajectory", 
			new RunRamseteTrajectory(drivetrain, calibrateTrajectory()));

        return chooser; 
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

}
