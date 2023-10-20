package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.*;
import frc.robot.commands.ArmCommands.*;
import frc.robot.commands.DrivetrainCommands.*;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.subsystems.*;

public final class AutonomousRoutines {
	public static SendableChooser<Command> createAutonomousChooser(final Drivetrain drivetrain, final Elevator elevator, final Arm arm, final Intake intake) {
		final SendableChooser<Command> chooser = new SendableChooser<>();

		chooser.addOption(
			"shoot high, drive out of community",
			new SequentialCommandGroup(
				new InitializeElevator(elevator),
				new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight),
				new ArmGoToPosition(arm, ArmConstants.highCubeAuto),
				new WaitCommand(.5),
				//new DriveDistance(.3, DrivetrainConstants.honeToHighDistance, drivetrain),
				new InstantCommand(()-> intake.setOutput(IntakeConstants.shootConePower), intake),
				new WaitCommand(.5),
				new InstantCommand(()-> intake.setOutput(0), intake),
				//new DriveDistance(-.3, -1 * DrivetrainConstants.honeToHighDistance, drivetrain),
				new StashIntake(elevator, arm),
				new DriveDistance(-.5, -3.5, drivetrain)
			)
		);

		chooser.addOption(
			"shoot high, balance",
			new SequentialCommandGroup(
				new InitializeElevator(elevator),
				new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight),
				new ArmGoToPosition(arm, ArmConstants.highCubeAuto),
				new WaitCommand(.4),
				//new DriveDistance(.3, DrivetrainConstants.honeToHighDistance, drivetrain),
				new InstantCommand(()-> intake.setOutput(IntakeConstants.shootConePower), intake),
				new WaitCommand(.5),
				new InstantCommand(()-> intake.setOutput(0), intake),
				//new DriveDistance(-.3, -1 * DrivetrainConstants.honeToHighDistance, drivetrain),
				new StashIntake(elevator, arm),
				new DriveDistance(-.5, -1.2, drivetrain),
				new Balance(drivetrain, false, 15000)
			)
		);

		chooser.setDefaultOption(
			"shoot high, drive over, balance",
			new SequentialCommandGroup(
				new InitializeElevator(elevator),
				new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight),
				new ArmGoToPosition(arm, ArmConstants.highCubeAuto),
				new WaitCommand(.4),
				new InstantCommand(()-> intake.setOutput(IntakeConstants.shootConePower), intake),
				new WaitCommand(.5),
				new InstantCommand(()-> intake.setOutput(0), intake),
				//new DriveDistance(-.3, -1 * DrivetrainConstants.honeToHighDistance, drivetrain),
				new ParallelCommandGroup(
					new StashIntake(elevator, arm),
					new InstantCommand(() -> Log.writeln("stashed")),
					new SequentialCommandGroup(new WaitCommand(1), new DriveDistance(-.35, -3, drivetrain))),
				new WaitCommand(.75),
				new DriveDistance(.35, 1.8, drivetrain),
				new Balance(drivetrain, false, 15000)
			)
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
