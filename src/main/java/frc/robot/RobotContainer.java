package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.POVSelector;
import frc.robot.commands.DrivetrainCommands.BalanceAUX;
import frc.robot.commands.DrivetrainCommands.BalancePID;
import frc.robot.commands.DrivetrainCommands.BalanceRollPID;
import frc.robot.commands.DrivetrainCommands.DriveTime;
import frc.robot.commands.DrivetrainCommands.GenerateTrajectory;
import frc.robot.commands.DrivetrainCommands.OrchestraPlayer;
import frc.robot.commands.DrivetrainCommands.RunDynamicRamseteTrajectory;
import frc.robot.commands.DrivetrainCommands.RunRamseteTrajectory;
import frc.robot.commands.POVSelector.Tree;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DynamicTrajectory;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Log;
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
	public final Drivetrain drivetrain = new Drivetrain(this.transmission::getGearState);

	// XBox Controllers
	private final XboxController driverController = new XboxController(0);
	private final DriverOI driverOI = new DriverOI(this.driverController);

	// Create SmartDashboard chooser for autonomous routines
	private final SendableChooser<Command> chooser = new SendableChooser<>();

	// Used for dynamic trajectories
	// public static Trajectory dynamicTrajectory = new Trajectory();
	public final DynamicTrajectory dynamicTrajectory = new DynamicTrajectory(this.drivetrain);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		this.configureAutoChooser();

		// Configure default commands, button bindings, and shuffleboard
		this.configureSubsystems();
		SmartDashboard.putNumber("Tag6 X", FieldConstants.aprilTags.get(6).toPose2d().getX());
		SmartDashboard.putNumber("Tag6 Y", FieldConstants.aprilTags.get(6).toPose2d().getY());
	}

	/**
	 * Configure all subsystems with their default command, button commands,
	 * and Shuffleboard output
	 */
	private void configureSubsystems() {
		this.configureDrivetrain();
	}

	public void configureDrivetrain() {
		// Configure default commands
		// Set the default drive command to split-stick arcade drive
		// A split-stick arcade command, with forward/backward controlled by the left
		// stick, and turning controlled by the right.
		this.drivetrain.setDefaultCommand(
			new RunCommand(
				() -> this.drivetrain.diffDrive.arcadeDrive(
					this.driverOI.getMoveSupplier().getAsDouble() * DrivetrainConstants.arcadeDriveMultiplier,
					this.driverOI.getRotateSupplier().getAsDouble() * DrivetrainConstants.arcadeDriveMultiplier
				),
				this.drivetrain
			)
		);

		// Configure button commands
		this.driverOI.getShiftLowButton().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.driverOI.getShiftHighButton().onTrue(new InstantCommand(this.transmission::setHigh, this.transmission));
		this.driverOI.getOrchestraButton().whileTrue(
			new OrchestraPlayer(
				this.drivetrain,
				Filesystem.getDeployDirectory().toPath().resolve("homedepot.chrp").toString()
			)
		);
		this.driverOI.getBalanceButton().whileTrue(BalancePID.manual(this.drivetrain));
		this.driverOI.getRollButton().whileTrue(BalanceRollPID.manual(this.drivetrain));
		this.driverOI.getBalanceAuxButton().whileTrue(BalanceAUX.manual(this.drivetrain));
		this.driverOI.getResetGyroButton().onTrue(new InstantCommand(this.drivetrain::zeroGyro, this.drivetrain));

		this.driverOI.getTestButton().toggleOnTrue(new POVSelector(
			this.driverOI,
			path -> Log.writeln("POV Selector step: ", String.join(",", path)),
			(str, path) -> Log.writeln("POV Selector finish: ", str, " (", String.join(",", path), ')'),
			new Tree("Deposit",
				new Tree("Center Station",
					new Tree("Center Position",
						new Tree("Top", new int[] { 2, 2, 3 }),
						new Tree("Middle", new int[] { 2, 2, 2 }),
						new Tree("Bottom", new int[] { 2, 2, 1 }),
						new Tree("Top", new int[] { 2, 2, 3 })
					),
					new Tree("Right Position",
						new Tree("Top", new int[] { 2, 3, 3 }),
						new Tree("Middle", new int[] { 2, 3, 2 }),
						new Tree("Bottom", new int[] { 2, 3, 1 }),
						new Tree("Top", new int[] { 2, 3, 3 })
					),
					new Tree(),
					new Tree("Left Position",
						new Tree("Top", new int[] { 2, 1, 3 }),
						new Tree("Middle", new int[] { 2, 1, 2 }),
						new Tree("Bottom", new int[] { 2, 1, 1 }),
						new Tree("Top", new int[] { 2, 1, 3 })
					)
				),
				new Tree("Right Station",
					new Tree("Center Position",
						new Tree("Top", new int[] { 3, 2, 3 }),
						new Tree("Middle", new int[] { 3, 2, 2 }),
						new Tree("Bottom", new int[] { 3, 2, 1 }),
						new Tree("Top", new int[] { 3, 2, 3 })
					),
					new Tree("Right Position",
						new Tree("Top", new int[] { 3, 3, 3 }),
						new Tree("Middle", new int[] { 3, 3, 2 }),
						new Tree("Bottom", new int[] { 3, 3, 1 }),
						new Tree("Top", new int[] { 3, 3, 3 })
					),
					new Tree(),
					new Tree("Left Position",
						new Tree("Top", new int[] { 3, 1, 3 }),
						new Tree("Middle", new int[] { 3, 1, 2 }),
						new Tree("Bottom", new int[] { 3, 1, 1 }),
						new Tree("Top", new int[] { 3, 1, 3 })
					)
				),
				new Tree(),
				new Tree("Left Station",
					new Tree("Center Position",
						new Tree("Top", new int[] { 1, 2, 3 }),
						new Tree("Middle", new int[] { 1, 2, 2 }),
						new Tree("Bottom", new int[] { 1, 2, 1 }),
						new Tree("Top", new int[] { 1, 2, 3 })
					),
					new Tree("Right Position",
						new Tree("Top", new int[] { 1, 3, 3 }),
						new Tree("Middle", new int[] { 1, 3, 2 }),
						new Tree("Bottom", new int[] { 1, 3, 1 }),
						new Tree("Top", new int[] { 1, 3, 3 })
					),
					new Tree(),
					new Tree("Left Position",
						new Tree("Top", new int[] { 1, 1, 3 }),
						new Tree("Middle", new int[] { 1, 1, 2 }),
						new Tree("Bottom", new int[] { 1, 1, 1 }),
						new Tree("Top", new int[] { 1, 1, 3 })
					)
				)
			)
		));
		
		// this.driverOI.getGoToTag6Button().onTrue(new RunDynamicRamseteTrajectory(this.drivetrain, 
		// 		() -> this.drivetrain.generateTrajectory(FieldConstants.tag6)));

	}


	private void configureAutoChooser() {
		// this.chooser.setDefaultOption("testing dropoff", new RunRamseteTrajectory(this.drivetrain, 
		// 							navigateToDropoff(FieldConstants.tag6, 1)));

		this.chooser.setDefaultOption("Test Dropoff", 
			new RunDynamicRamseteTrajectory(this.drivetrain, () -> this.drivetrain.generateTrajectory(FieldConstants.tag6)));


		this.chooser.addOption(
			"Back up to balance",
			new SequentialCommandGroup(
				new WaitCommand(.2),
				new RunRamseteTrajectory(this.drivetrain, this.loadTrajectory("BackUpToBalance")),
				// Todo: find right time/speed to get onto teeter totter
				new DriveTime(-.4, .5, this.drivetrain),
				BalanceRollPID.auto(this.drivetrain, 7000),
				BalancePID.auto(this.drivetrain, 7000)
			)
		);
		this.chooser.addOption(
			"Curve right around Charging Station and balance",
			new SequentialCommandGroup(
				new WaitCommand(.1),
				new RunRamseteTrajectory(this.drivetrain, this.loadTrajectory("Auto1")),
				// Todo: find right time/speed to get onto teeter totter
				// new DriveTime(-.4, .5, this.drivetrain),
				BalanceRollPID.auto(this.drivetrain, 7000),
				BalancePID.auto(this.drivetrain, 7000)
			)
		);
		this.chooser.addOption(
			"test",
			new SequentialCommandGroup(
				new WaitCommand(.1),
				new RunRamseteTrajectory(this.drivetrain, this.loadTrajectory("BackUpToBalance"))
			)
		);
		this.chooser.addOption(
			"testing",
			new SequentialCommandGroup(
				new WaitCommand(.1),
				new DriveTime(.5, 2, this.drivetrain)
			)
		);
		this.chooser.addOption(
			"backupbalance",
			new SequentialCommandGroup(
				new WaitCommand(.1),
				new RunRamseteTrajectory(this.drivetrain, this.loadTrajectory("BackUpToBalance")),
				BalanceRollPID.auto(this.drivetrain, 7000),
				BalancePID.auto(this.drivetrain, 7000)
			)
		);

		this.chooser.addOption(
			"auto 3",
			new SequentialCommandGroup(
				new WaitCommand(.1),
				new RunRamseteTrajectory(this.drivetrain, this.loadTrajectory("Auto3"))
				// Todo: find right time/speed to get onto teeter totter
				// new DriveTime(-.4, .5, this.drivetrain),
				// BalanceRollPID.auto(this.drivetrain, 7000),
				// BalancePID.auto(this.drivetrain, 7000)
			)
		);

		this.chooser.addOption("Calibrate Trajectory", new RunRamseteTrajectory(this.drivetrain, calibrateTrajectory()));
		SmartDashboard.putData("AutoRoutineChooser", this.chooser);
	}

	public Trajectory loadTrajectory(String trajectoryJSON) {
		Path trajectoryPath = Filesystem
			.getDeployDirectory()
			.toPath()
			.resolve("paths/" + trajectoryJSON + ".wpilib.json");
		
		try {
			return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			Log.error(ex);
			return null;
		}
	}

	/**
	 * Drives a straight line 2 meters so as you can calibrate your Romi
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
			new Pose2d(4.0, 0.0, new Rotation2d(0)), // left
			DrivetrainConstants.kTrajectoryConfig);
  	}

	/**
	 * 
	 * @param endPose pose where robot should end
	 * @param direction 0 for left, 1 for right, 2 for robot to decide
	 * @return
	 */
	public Trajectory navigateToDropoff(Pose2d endPose, int direction){
		Trajectory trajectory;
		Pose2d startPose;
		
		startPose = this.drivetrain.getEstimatedPose();
		startPose = new Pose2d(5.0,4.0, new Rotation2d(3.1));
		// startPose = new Pose2d(0.0,0.0, new Rotation2d());
		SmartDashboard.putNumber("Start Pose X", startPose.getX());
		SmartDashboard.putNumber("Start Pose Y", startPose.getY());
		SmartDashboard.putNumber("Start Pose Theta", startPose.getRotation().getDegrees());

		SmartDashboard.putNumber("End Pose X", endPose.getX());
		SmartDashboard.putNumber("End Pose Y", endPose.getY());
		SmartDashboard.putNumber("End Pose Theta", endPose.getRotation().getDegrees());

		DriverStation.Alliance color = DriverStation.getAlliance();

		// trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of (new Translation2d(1, 0)),
		// 									new Pose2d(2, 0, new Rotation2d(0)), DrivetrainConstants.kTrajectoryConfig);
		
		if(color == DriverStation.Alliance.Red){
			// for red, left and right
			//if direction is specified left, or direction is unspecified and Y is on left side of field...
			if(direction == 0 || ((direction == 2 ) && (startPose.getY() <= (DrivetrainConstants.fieldWidthYMeters / 2)))){
				trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
				//List.of(DrivetrainConstants.leftRedWaypoint1, DrivetrainConstants.leftRedWaypoint2), \
				List.of(),
				endPose, DrivetrainConstants.kTrajectoryConfig);
			} else {
				trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
					//List.of(DrivetrainConstants.rightRedWaypoint1, DrivetrainConstants.rightRedWaypoint2), 
					List.of(),
					endPose, DrivetrainConstants.kTrajectoryConfig);
			}
		} else {
			// for blue, left and right
			if(direction == 0 || ((direction == 2 ) && (startPose.getY() >= (DrivetrainConstants.fieldWidthYMeters / 2)))){
				trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
					//List.of(DrivetrainConstants.leftBlueWaypoint1, DrivetrainConstants.leftBlueWaypoint2), 
					List.of(),
					endPose, DrivetrainConstants.kTrajectoryConfig);
			} else {
				trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
					//List.of(DrivetrainConstants.rightBlueWaypoint1, DrivetrainConstants.rightBlueWaypoint2), 
					List.of(),
					endPose, DrivetrainConstants.kTrajectoryConfig);
			}
		}

		SmartDashboard.putNumber("Waypoint1 X", FieldConstants.Waypoints.rightBlue1.getX());
		SmartDashboard.putNumber("Waypoint Y", FieldConstants.Waypoints.rightBlue1.getY());

		return trajectory;
	}

	public Command getAutonomousCommand() {
		return this.chooser.getSelected();
	}
}
