package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.DrivetrainCommands.BalancePID;
import frc.robot.commands.DrivetrainCommands.BalanceRollPID;
import frc.robot.commands.DrivetrainCommands.DriveTime;
import frc.robot.commands.DrivetrainCommands.OrchestraPlayer;
import frc.robot.commands.DrivetrainCommands.RunRamseteTrajectory;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transmission;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The Robot's Subsystems
	public final Transmission transmission = new Transmission();
	public final Drivetrain drivetrain = new Drivetrain(transmission::getGearState);

	// XBox Controllers
	private final XboxController driverController = new XboxController(0);
	private final DriverOI driverOI = new DriverOI(driverController);

	// Create SmartDashboard chooser for autonomous routines
	private final SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		configureAutoChooser();

		// Configure default commands, button bindings, and shuffleboard
		configureSubsystems();
	}

	/**
	 * Configure all subsystems with their default command, button commands,
	 * and Shuffleboard output
	 */
	private void configureSubsystems() {
		configureDrivetrain();
	}

	public void configureDrivetrain() {
		// Configure default commands
		// Set the default drive command to split-stick arcade drive
		// A split-stick arcade command, with forward/backward controlled by the left
		// stick, and turning controlled by the right.
		drivetrain.setDefaultCommand(
			new RunCommand(
				() -> drivetrain.diffDrive.arcadeDrive(
					driverOI.getMoveSupplier().getAsDouble() * DrivetrainConstants.arcadeDriveMultiplier,
					driverOI.getRotateSupplier().getAsDouble() * DrivetrainConstants.arcadeDriveMultiplier
				),
				drivetrain
			)
		);

		// Configure button commands
		driverOI.getShiftLowButton().onTrue(new InstantCommand(transmission::setLow, transmission));
		driverOI.getShiftHighButton().onTrue(new InstantCommand(transmission::setHigh, transmission));
		driverOI.getOrchestraButton().whileTrue(
			new OrchestraPlayer(
				drivetrain,
				Filesystem.getDeployDirectory().toPath().resolve("homedepot.chrp").toString()
			)
		);
		driverOI.getBalanceButton().whileTrue(BalancePID.manual(this.drivetrain));
		driverOI.getRollButton().whileTrue(BalanceRollPID.manual(this.drivetrain));
		driverOI.getResetGyroButton().onTrue(new InstantCommand(drivetrain::zeroGyro, drivetrain));
	}


	private void configureAutoChooser() {
		chooser.setDefaultOption("-", new InstantCommand(() -> this.drivetrain.halt()));
		chooser.addOption(
			"Back up to balance",
			new SequentialCommandGroup(
				new WaitCommand(.2),
				new RunRamseteTrajectory(drivetrain, loadTrajectory("BackUpToBalance")),
				// Todo: find right time/speed to get onto teeter totter
				new DriveTime(-.4, .5, drivetrain),
				BalanceRollPID.auto(drivetrain, 7000),
				BalancePID.auto(drivetrain, 7000)
			)
		);
		chooser.addOption(
			"Curve right around Chargin Station and balance",
			new SequentialCommandGroup(
				new WaitCommand(.1),
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Auto1")),
				// Todo: find right time/speed to get onto teeter totter
				// new DriveTime(-.4, .5, this.drivetrain),
				BalanceRollPID.auto(drivetrain, 7000),
				BalancePID.auto(drivetrain, 7000)
			)
		);
		chooser.addOption(
			"test",
			new SequentialCommandGroup(
				new WaitCommand(.1),
				new RunRamseteTrajectory(drivetrain, loadTrajectory("BackUpToBalance"))
			)
		);
		chooser.addOption(
			"testing",
			new SequentialCommandGroup(
				new WaitCommand(.1),
				new DriveTime(.5, 2, drivetrain)
			)
		);
		chooser.addOption(
			"backupbalance",
			new SequentialCommandGroup(
				new WaitCommand(.1),
				new RunRamseteTrajectory(drivetrain, loadTrajectory("BackUpToBalance")),
				BalanceRollPID.auto(drivetrain, 7000),
				BalancePID.auto(drivetrain, 7000)
			)
		);

		chooser.addOption(
			"auto2",
			new SequentialCommandGroup(
				new WaitCommand(.1),
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Auto3")),
				// Todo: find right time/speed to get onto teeter totter
				// new DriveTime(-.4, .5, this.drivetrain),
				BalanceRollPID.auto(drivetrain, 7000),
				BalancePID.auto(drivetrain, 7000)
			)
		);

		SmartDashboard.putData("AutoRoutineChooser", chooser);
	}

	public Trajectory loadTrajectory(String trajectoryJSON) {
		Trajectory trajectory = new Trajectory();

		try {
			Path trajectoryPath = Filesystem
				.getDeployDirectory()
				.toPath()
				.resolve("paths/" + trajectoryJSON + ".wpilib.json");
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open Trajectory:" + trajectoryJSON, ex.getStackTrace());
		}
		return trajectory;
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}
