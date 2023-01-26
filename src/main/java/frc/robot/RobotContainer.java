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
import frc.robot.commands.DrivetrainCommands.BalanceAUX;
import frc.robot.commands.DrivetrainCommands.BalancePID;
import frc.robot.commands.DrivetrainCommands.BalanceRollPID;
import frc.robot.commands.DrivetrainCommands.DriveTime;
import frc.robot.commands.DrivetrainCommands.GenerateTrajectory;
import frc.robot.commands.DrivetrainCommands.OrchestraPlayer;
import frc.robot.commands.DrivetrainCommands.RunDynamicRamseteTrajectory;
import frc.robot.commands.DrivetrainCommands.RunRamseteTrajectory;
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
	public final Drivetrain drivetrain = new Drivetrain(transmission::getGearState);

	// XBox Controllers
	private final XboxController driverController = new XboxController(0);
	private final DriverOI driverOI = new DriverOI(driverController);

	// Create SmartDashboard chooser for autonomous routines
	private final SendableChooser<Command> chooser = new SendableChooser<>();

	// Used for dynamic trajectories
	// public static Trajectory dynamicTrajectory = new Trajectory();
	public final DynamicTrajectory dynamicTrajectory = new DynamicTrajectory(drivetrain);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		configureAutoChooser();

		// Configure default commands, button bindings, and shuffleboard
		configureSubsystems();
		SmartDashboard.putNumber("Tag6 X",FieldConstants.aprilTags.get(6).toPose2d().getX());
		SmartDashboard.putNumber("Tag6 Y",FieldConstants.aprilTags.get(6).toPose2d().getY());
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
		driverOI.getBalanceAuxButton().whileTrue(BalanceAUX.manual(this.drivetrain));
		driverOI.getResetGyroButton().onTrue(new InstantCommand(drivetrain::zeroGyro, drivetrain));
		
		// driverOI.getGoToTag6Button().onTrue(new RunDynamicRamseteTrajectory(this.drivetrain, 
		// 		() -> this.drivetrain.generateTrajectory(FieldConstants.tag6)));

	}


	private void configureAutoChooser() {
		// chooser.setDefaultOption("testing dropoff", new RunRamseteTrajectory(this.drivetrain, 
		// 							navigateToDropoff(FieldConstants.tag6, 1)));

		chooser.setDefaultOption("Test Dropoff", 
			new RunDynamicRamseteTrajectory(this.drivetrain, () -> this.drivetrain.generateTrajectory(FieldConstants.tag6)));


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
			"Curve right around Charging Station and balance",
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
			"auto 3",
			new SequentialCommandGroup(
				new WaitCommand(.1),
				new RunRamseteTrajectory(drivetrain, loadTrajectory("Auto3"))
				// Todo: find right time/speed to get onto teeter totter
				// new DriveTime(-.4, .5, this.drivetrain),
				// BalanceRollPID.auto(drivetrain, 7000),
				// BalancePID.auto(drivetrain, 7000)
			)
		);

		chooser.addOption("Calibrate Trajectory", new RunRamseteTrajectory(drivetrain, calibrateTrajectory()));
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
			Log.error(ex);
		}
		return trajectory;
	}

	/**
   * Drives a straight line 2 meters so as you can calibrate your Romi
   * You should make sure that the robot ends up right on the 2 meter mark.
   *
   */
  public Trajectory calibrateTrajectory() {
    
    // Note that all coordinates are in meters, and follow NWU conventions.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // List.of(new Translation2d(2.0, 0.0)) 
		List.of(),
        new Pose2d(4.0, 0.0, new Rotation2d(0)), // left
        DrivetrainConstants.kTrajectoryConfig);

    return trajectory;
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

		trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
				List.of(FieldConstants.Waypoints.rightBlue1),
				// List.of(),
				endPose, DrivetrainConstants.kTrajectoryConfig);

		return trajectory;
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}
