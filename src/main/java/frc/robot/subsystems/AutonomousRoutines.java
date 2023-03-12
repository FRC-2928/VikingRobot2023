package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.ArmCommands.ArmGoToPosition;
import frc.robot.commands.DrivetrainCommands.Balance;
import frc.robot.commands.DrivetrainCommands.DriveDistance;
import frc.robot.commands.DrivetrainCommands.RunRamseteTrajectory;
import frc.robot.commands.DrivetrainCommands.TurnToPid;
import frc.robot.commands.ElevatorCommands.ElevatorGoToHeight;
import frc.robot.commands.ElevatorCommands.InitializeElevator;
import frc.robot.commands.ElevatorCommands.StashIntake;
import frc.robot.subsystems.TrajectoryRunner.Direction;

public final class AutonomousRoutines {
    public static SendableChooser<Command> createAutonomousChooser(Drivetrain drivetrain, Elevator elevator, Arm arm, Intake intake) {
		SendableChooser<Command> chooser = new SendableChooser<>();

		chooser.addOption(
			"Tag7/2 Balance",
			new SequentialCommandGroup(
				//new DriveTime(-.6, 4.5, drivetrain),
				Balance.timed(drivetrain, 1000),Balance.manual(drivetrain)
			)
		);

		chooser.setDefaultOption("shoot high and drive",
			new SequentialCommandGroup(
				new InitializeElevator(elevator),
				new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight),
				new ArmGoToPosition(arm, ArmConstants.highPosition),
				//new DriveDistance(.3, DrivetrainConstants.honeToHighDistance, drivetrain),
				new InstantCommand(()-> intake.setOutput(IntakeConstants.shootConePower), intake),
				new WaitCommand(.5),
				new InstantCommand(()-> intake.setOutput(0), intake),
				//new DriveDistance(-.3, -1 * DrivetrainConstants.honeToHighDistance, drivetrain),
				new StashIntake(elevator, arm),
				new DriveDistance(-.5, -3.5, drivetrain)
			)
		);

		chooser.addOption("shoot cone high and drive?",
			new SequentialCommandGroup(
				new InitializeElevator(elevator),
				new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight),
				new ArmGoToPosition(arm, ArmConstants.highPosition),
				new DriveDistance(.3, DrivetrainConstants.toHighDistance, drivetrain),
				new InstantCommand(()-> intake.setOutput(IntakeConstants.shootConePower), intake),
				new WaitCommand(.5),
				new InstantCommand(()-> intake.setOutput(0), intake),
				new DriveDistance(-.3, -1 * DrivetrainConstants.toHighDistance, drivetrain),
				new StashIntake(elevator, arm),
				new DriveDistance(-.5, -3.5, drivetrain)
			)
		);

		chooser.addOption("shoot high don't drive",
			new SequentialCommandGroup(
				new InitializeElevator(elevator),
				new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight),
				new ArmGoToPosition(arm, ArmConstants.highPosition),
				//new DriveDistance(.3, DrivetrainConstants.honeToHighDistance, drivetrain),
				new InstantCommand(()-> intake.setOutput(IntakeConstants.shootConePower), intake),
				new WaitCommand(.5),
				new InstantCommand(()-> intake.setOutput(0), intake),
				//new DriveDistance(-.3, -1 * DrivetrainConstants.honeToHighDistance, drivetrain),
				new StashIntake(elevator, arm)
			)
		);

		chooser.addOption("shoot high and balance?",
			new SequentialCommandGroup(
				new InitializeElevator(elevator),
				new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight),
				new ArmGoToPosition(arm, ArmConstants.highPosition),
				//new DriveDistance(.3, DrivetrainConstants.honeToHighDistance, drivetrain),
				new InstantCommand(()-> intake.setOutput(IntakeConstants.shootConePower), intake),
				new WaitCommand(.5),
				new InstantCommand(()-> intake.setOutput(0), intake),
				//new DriveDistance(-.3, -1 * DrivetrainConstants.honeToHighDistance, drivetrain),
				new StashIntake(elevator, arm),
				new DriveDistance(-.5, -1, drivetrain),
				new Balance(drivetrain, false, 15000))
							);	
		
		chooser.addOption("shoot high drive over and balance?", 
							new SequentialCommandGroup(
									new InitializeElevator(elevator),
									new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight),
									new ArmGoToPosition(arm, ArmConstants.highPosition),
									//new DriveDistance(.3, DrivetrainConstants.honeToHighDistance, drivetrain),
									new InstantCommand(()-> intake.setOutput(IntakeConstants.shootConePower), intake),
									new WaitCommand(.5),
									new InstantCommand(()-> intake.setOutput(0), intake),
									//new DriveDistance(-.3, -1 * DrivetrainConstants.honeToHighDistance, drivetrain),
									new StashIntake(elevator, arm),
									new DriveDistance(-.35, -2, drivetrain),
									new DriveDistance(.5, 1, drivetrain),
									new Balance(drivetrain, false, 15000)
							)
		);	

		chooser.addOption("shoot high drive over and balance?",
			new SequentialCommandGroup(
				new InitializeElevator(elevator),
				new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight),
				new ArmGoToPosition(arm, ArmConstants.highPosition),
				//new DriveDistance(.3, DrivetrainConstants.honeToHighDistance, drivetrain),
				new InstantCommand(()-> intake.setOutput(IntakeConstants.shootConePower), intake),
				new WaitCommand(.5),
				new InstantCommand(()-> intake.setOutput(0), intake),
				//new DriveDistance(-.3, -1 * DrivetrainConstants.honeToHighDistance, drivetrain),
				new StashIntake(elevator, arm),
				new DriveDistance(-.35, -2, drivetrain),
				new DriveDistance(.5, 1, drivetrain),
				new Balance(drivetrain, false, 15000)
			)
		);

		chooser.addOption("shoot middddd",
			new SequentialCommandGroup(
				new InitializeElevator(elevator),
				new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight),
				new ArmGoToPosition(arm, ArmConstants.midPosition),
				//new DriveDistance(.3, DrivetrainConstants.honeToHighDistance, drivetrain),
				new InstantCommand(()-> intake.setOutput(IntakeConstants.shootConePower * .75), intake),
				new WaitCommand(.5),
				new InstantCommand(()-> intake.setOutput(0), intake),
				//new DriveDistance(-.3, -1 * DrivetrainConstants.honeToHighDistance, drivetrain),
				new StashIntake(elevator, arm),
				new DriveDistance(-.5, -3.5, drivetrain)
			)
		);

		// once drive distance works
		// chooser.setDefaultOption("just shoot",
		// 	new SequentialCommandGroup(
		// 		new InitializeElevator(elevator),
		// 		new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight),
		// 		new ArmGoToPosition(arm, ArmConstants.highPosition),
		// 		new DriveDistance(.3, DrivetrainConstants.honeToHighDistance, drivetrain),
		// 		new InstantCommand(()-> intake.setOutput(IntakeConstants.shootCubePower), intake),
		// 		new WaitCommand(.5),
		// 		new InstantCommand(()-> intake.setOutput(0), intake),
		// 		new DriveDistance(-.3, -1 * DrivetrainConstants.honeToHighDistance, drivetrain),
		// 		new StashIntake(elevator, arm)
		// 	)
		// );

		//set in pathweaver to go forward a little bit
		chooser.addOption(
			"test pathweaver",
			new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("test"))
		);

		chooser.addOption(
			"test drive distance",
			new DriveDistance(.3, 0.25, drivetrain)
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

		chooser.addOption(
			"Tag3 Routines1",
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

		chooser.addOption(
			"Tag8 Routines1",
			new SequentialCommandGroup(
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Tag8-Rotate4")),
				new TurnToPid(0, drivetrain),
				new DriveDistance(.5, .5, drivetrain),
				new TurnToPid(180, drivetrain),
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Rotate4-AprilTag8")),
				TrajectoryRunner.generateRamseteCommand(drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(drivetrain, Direction.Center))
			)
		);

		chooser.addOption(
			"Tag8/3 Balance",
			new SequentialCommandGroup(
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Tag8-Around-Balance")),
				Balance.timed(drivetrain, 1000),Balance.manual(drivetrain)
			)
		);

		chooser.addOption(
			"Tag6/1 Balance",
			new SequentialCommandGroup(
				new RunRamseteTrajectory(drivetrain, TrajectoryRunner.loadTrajectory("Tag6-Around-Balance")),
				Balance.timed(drivetrain, 1000),Balance.manual(drivetrain)
			)
		);

		chooser.addOption(
			"Calibrate Trajectory",
			new RunRamseteTrajectory(drivetrain, AutonomousRoutines.calibrateTrajectory())
		);

        return chooser;
	}

    /**
	 * Drives a straight line 4 meters so as you can calibrate your Romi
	 * You should make sure that the robot ends up right on the 2 meter mark.
	 */
  	public static Trajectory calibrateTrajectory() {
		return TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// List.of(new Translation2d(2.0, 0.0))
			List.of(),
			new Pose2d(4, 0.0, new Rotation2d(0)), // left
			AutoConstants.trajectoryConfig
		);
  	}
}
