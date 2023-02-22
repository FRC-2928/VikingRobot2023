package frc.robot;

import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.POVSelector;
import frc.robot.commands.DrivetrainCommands.BalanceAUX;
import frc.robot.commands.DrivetrainCommands.BalancePID;
import frc.robot.commands.POVSelector.Tree;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.AutoRoutines;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Log;
import frc.robot.subsystems.TrajectoryRunner;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Transmission;
import frc.robot.subsystems.TrajectoryRunner.Direction;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// Mechanism Subsystems
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// Direction around the Charger Station
	// public static enum Direction {
	// 	Left,
	// 	Right,
	// 	Center
	// }

	// The Robot's Subsystems
	public final Transmission transmission = new Transmission();
	public final Drivetrain drivetrain = new Drivetrain(this.transmission::getGearState);
	// public final Intake intake = new Intake();
	// public final Elevator elevator = new Elevator();
	// public final Arm arm = new Arm();

	// XBox Controllers
	private final XboxController driverController = new XboxController(0);
	private final XboxController operatorController = new XboxController(1);
	private final DriverOI driverOI = new DriverOI(this.driverController);
	private final OperatorOI operatorOI = new OperatorOI(this.operatorController);

	// Create SmartDashboard chooser for autonomous routines
	private AutoRoutines autos;
	private SendableChooser<Command> chooser = new SendableChooser<>();

	public static DriverStation.Alliance alliance = DriverStation.Alliance.Blue;

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

	public static void setAlliance(Alliance alliance) {
		RobotContainer.alliance = alliance;
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

		//operator buttons
		// this.operatorOI.getRunIntakeButton().onTrue(new InstantCommand(() -> intake.setOutput(IntakeConstants.intakePower)));
		// this.operatorOI.getShootIntakeButton().onTrue(new InstantCommand(() -> intake.setOutput(IntakeConstants.shootPower)));
		// this.operatorOI.getStopIntakeButton().onTrue(new InstantCommand(() -> intake.setOutput(0)));
		// default command should run only in absence of other commands - 
			//shouldn't be a problem for these to be default even though they're backup
				// CHECK THOUGH
		// this.elevator.setDefaultCommand(new RunCommand(() -> elevator.setPower(m_operatorOI.getElevatorSupplier()), elevator));
		// this.arm.setDefaultCommand(new RunCommand(() -> elevator.setPower(m_operatorOI.getElevatorSupplier()), arm));
		
		// this.operatorOI.getHigh().onTrue(new MoveElevatorAndArm(elevator, arm, ElevatorConstants.highHeight, ArmConstants.highHeight));
		// this.operatorOI.getMid().onTrue(new MoveElevatorAndArm(elevator, arm, ElevatorConstants.midHeight, ArmConstants.midHeight));
		// this.operatorOI.getLow().onTrue(new MoveElevatorAndArm(elevator, arm, ElevatorConstants.lowHeight, ArmConstants.lowHeight));

		// Configure gear shifting
		if(RobotBase.isReal()) {
			this.driverOI.getShiftLowButton().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		} else {
			// Uses the X button to test ApproachTag in simulation
			this.driverOI.getShiftLowButton().onTrue(
				TrajectoryRunner.generateRamseteCommand(drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(drivetrain, Direction.Center)));
		}		
		this.driverOI.getShiftHighButton().onTrue(new InstantCommand(this.transmission::setHigh, this.transmission));
		
		// this.driverOI.getOrchestraButton().whileTrue(
		// 	new OrchestraPlayer(
		// 		this.drivetrain,
		// 		Filesystem.getDeployDirectory().toPath().resolve("homedepot.chrp").toString()
		// 	)
		// );

		//this.driverOI.getBalanceButton().whileTrue(BalancePID.manual(this.drivetrain));
		//this.driverOI.getRollButton().whileTrue(BalanceRollPID.manual(this.drivetrain));
		//this.driverOI.getApproachTagButton().whileTrue(ApproachTag.manual(this.drivetrain));
		// this.driverOI.getBalanceAuxButton().whileTrue(BalanceAUX.manual(this.drivetrain));
		this.driverOI.getBalanceAuxButton().onTrue((new SequentialCommandGroup(new BalancePID(drivetrain, false, 10),BalanceAUX.manual(drivetrain))));
		this.driverOI.getResetGyroButton().onTrue(new InstantCommand(() -> {
			this.drivetrain.zeroGyro();
			this.drivetrain.resetEncoders();
		}, this.drivetrain));
		this.driverOI.getHaltButton().onTrue(new InstantCommand(() -> {
			Log.writeln("[HALT]");
			this.drivetrain.halt();
			CommandScheduler.getInstance().cancelAll();
		}));

		// this.driverOI.getStartButton().onTrue(new InstantCommand(()->this.generateTrajectory(FieldConstants.tag6)));
		// this.driverOI.getStartButton().onTrue(this.generateRamseteCommand(() -> {
		// 	Log.writeln("dyn traj");
		// 	return this.generateLocalTrajectory(Direction.Right);
		// }));

		this.driverOI.getApproachTagButton().toggleOnTrue(new POVSelector(
			this.driverOI,
			null,
			(dir, __) -> {
				CommandScheduler
					.getInstance()
					.schedule(TrajectoryRunner.generateRamseteCommand(drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(drivetrain, (Direction)dir)));
			},
			new Tree(
				"Select tag offset",
				new Tree("Center", Direction.Center),
				new Tree("Right", Direction.Right),
				new Tree(),
				new Tree("Left", Direction.Left)
			)
		));
		
	}

	private void configureAutoChooser() {
		autos = new AutoRoutines(drivetrain);
		chooser = autos.configureAutoChooser();
		SmartDashboard.putData("AutoRoutineChooser", chooser);
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
	// 		.andThen(new InstantCommand(() -> drivetrain.halt(), drivetrain));
	// } 

	/** 
	 * Load the trajectory from the roboRIOs deploy directory
	 * 
	 * @param trajectoryJSON name of trajectory to load
	 */
	// public Trajectory loadTrajectory(String trajectoryJSON) {
	// 	Path trajectoryPath = Filesystem
	// 		.getDeployDirectory()
	// 		.toPath()
	// 		.resolve("paths/output/" + trajectoryJSON + ".wpilib.json");
		
	// 	try {
	// 		return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
	// 	} catch (IOException ex) {
	// 		Log.error(ex);
	// 		return null;
	// 	}
	// }

	/**
	 * Drives a straight line 4 meters so as you can calibrate your Romi
	 * You should make sure that the robot ends up right on the 2 meter mark.
	 *
	 */
  	// public Trajectory calibrateTrajectory() {
	// 	// Note that all coordinates are in meters, and follow NWU conventions.
	// 	return TrajectoryGenerator.generateTrajectory(
	// 		// Start at the origin facing the +X direction
	// 		new Pose2d(0, 0, new Rotation2d(0)),
	// 		// List.of(new Translation2d(2.0, 0.0)) 
	// 		List.of(),
	// 		new Pose2d(4, 0.0, new Rotation2d(0)), // left
	// 		AutoConstants.kTrajectoryConfig);
  	// }

	// /**
	//  * Generates a dynamic trajectory starting at the current pose of the
	//  * robot, as determined by the Limelight looking at the AprilTags.
	//  * Additional waypoints may be added to navigate around field structures.
	//  * 
	//  * @param endPose pose where robot should end
	//  * 
	//  * @return The generated Trajectory object
	//  */
	// public Trajectory generateTrajectory(Pose2d endPose) {
    //     Pose2d startPose = this.drivetrain.getLimelightPoseRelative();

    //     Log.writeln("Generate start Pose: " + startPose);
    //     SmartDashboard.putNumber("Start Pose X", startPose.getX());
    //     SmartDashboard.putNumber("Start Pose Y", startPose.getY());
    //     SmartDashboard.putNumber("Start Pose Heading", startPose.getRotation().getDegrees());
    
    //     SmartDashboard.putNumber("End Pose X", endPose.getX());
    //     SmartDashboard.putNumber("End Pose Y", endPose.getY());
    //     SmartDashboard.putNumber("End Pose Heading", endPose.getRotation().getDegrees());
        
	// 	Trajectory trajectory;
	// 	List<Translation2d> waypoints = new ArrayList<>();

	// 	// Log.writeln("Alliance:" + alliance);
    //     // if(alliance == DriverStation.Alliance.Red){
    //     // 	// for red, left and right
    //     // 	//if direction is specified left, or direction is unspecified and Y is on left side of field...
    //     // 	if(direction == Direction.Left || ((direction == Direction.Unspecified ) && (drivetrain.isLeftOfChargingStation()))){
	// 	// 		Log.writeln("Red left");
    //     // 		trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
	// 	// 			List.of(FieldConstants.Waypoints.leftRed1, FieldConstants.Waypoints.leftRed2),
	// 	// 			endPose, DrivetrainConstants.kTrajectoryConfig);
    //     // 	} else {
	// 	// 		Log.writeln("Red right");
    //     // 		trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
	// 	// 			List.of(FieldConstants.Waypoints.rightRed1, FieldConstants.Waypoints.rightRed2), 
	// 	// 			endPose, DrivetrainConstants.kTrajectoryConfig);
    //     // 	}
    //     // } else {
    //     // 	// for blue, left and right
    //     // 	if(direction == Direction.Left || ((direction == Direction.Unspecified) && (drivetrain.isLeftOfChargingStation()))){
    //     // 		Log.writeln("Blue left");
	// 	// 		Log.writeln("CS Center" + FieldConstants.Community.chargingStationCenterY);
	// 	// 		if (startPose.getX() > FieldConstants.Waypoints.leftBlue1.getX()) {
	// 	// 			waypoints.add(FieldConstants.Waypoints.leftBlue1);
	// 	// 		}
	// 	// 		if (startPose.getX() > FieldConstants.Waypoints.leftBlue2.getX()) {
	// 	// 			waypoints.add(FieldConstants.Waypoints.leftBlue2);
	// 	// 		}				

	// 	// 		trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
	// 	// 			waypoints,
    //     // 			endPose, DrivetrainConstants.kTrajectoryConfig);
    //     // 	} else {
	// 	// 		Log.writeln("Blue right");
	// 	// 		if (startPose.getX() > FieldConstants.Waypoints.rightBlue1.getX()) {
	// 	// 			waypoints.add(FieldConstants.Waypoints.rightBlue1);
	// 	// 		}		
	// 	// 		if (startPose.getX() > FieldConstants.Waypoints.rightBlue2.getX()) {
	// 	// 			waypoints.add(FieldConstants.Waypoints.rightBlue2);
	// 	// 		} 		
				
    //     // 		trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
	// 	// 			waypoints,
    //     // 			endPose, DrivetrainConstants.kTrajectoryConfig);
    //     // 	}
    //     // }

    // 	// waypoints.add(new Translation2d(endPose.getX() + 2, endPose.getY() + 0.1));
	// 	trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
	// 				waypoints,
    //     			endPose, AutoConstants.kTrajectoryConfig);

	// 	Log.writeln("Initial Pose: " + trajectory.getInitialPose());
	// 	Log.writeln("Waypoints:" + waypoints);
	// 	Log.writeln("End Pose:" + endPose);

    //     this.printTrajectory(trajectory);

    //     return trajectory;
    // }

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
	// public Trajectory generateLocalTrajectory(Direction direction) {
		
	// 	Log.writeln("generateLocalTrajectory");
	// 	Trajectory trajectory = new Trajectory();
		
	// 	// Get the aprilTag that the robot is looking at
	// 	double aprilTagID = this.drivetrain.getAprilTagID();

	// 	// Default trajectory if no limelight target is to move back 0.5 meters
	// 	Pose2d startPose = this.drivetrain.getEncoderPose();
	// 	Pose2d endPose = startPose.plus(new Transform2d(new Translation2d(-0.5, 0), 
	// 									new Rotation2d()));
	// 	// Set the trajectory config to reversed.
	// 	TrajectoryConfig config = AutoConstants.kTrajectoryConfigReversed;
		
	// 	List<Translation2d> waypoints = new ArrayList<>();
	
	// 	if (this.drivetrain.hasNoLimelightTarget()) {			
	// 		Log.writeln("No Limelight target!");	
	// 		Log.writeln("Start Pose X ", startPose.getX());
	// 		Log.writeln("Start Pose Y ", startPose.getY());
	// 		Log.writeln("Start Pose Heading ", startPose.getRotation().getDegrees());
		
	// 		Log.writeln("End Pose X ", endPose.getX());
	// 		Log.writeln("End Pose Y ", endPose.getY());
	// 		Log.writeln("End Pose Heading ", endPose.getRotation().getDegrees());

	// 	} else if(!FieldConstants.aprilTags.containsKey((int)aprilTagID)){
	// 		Log.writeln("Invalid aprilTag! " + aprilTagID);	

	// 	} else {
	// 		// Get the aprilTag that the robot is looking at and it's pose relative to the tag.
	// 		startPose = this.drivetrain.getLimelightPoseRelative();
	// 		// Move forward config
	// 		config = AutoConstants.kTrajectoryConfig;

	// 		// Now get the pose
	// 		Pose2d tag = FieldConstants.aprilTags.get((int)aprilTagID).toPose2d();
	// 		switch(direction){
	// 			case Left:
	// 			// endPose = tag.plus(new Transform2d(new Translation2d(0.75, -Units.inchesToMeters(22.5)), new Rotation2d(Math.PI)));
	// 			endPose = tag.plus(FieldConstants.leftOffset);	
	// 			break;
	// 			case Right:
	// 			// endPose = tag.plus(new Transform2d(new Translation2d(0.75, Units.inchesToMeters(22.5)), new Rotation2d(Math.PI)));	
	// 			endPose = tag.plus(FieldConstants.rightOffset);	
	// 			break;
	// 			default:
	// 			// endPose = tag.plus(new Transform2d(new Translation2d(0.75, 0), new Rotation2d(Math.PI)));
	// 			endPose = tag.plus(FieldConstants.centerOffset);		
	// 			break;		
	// 		}
	
	// 	}

	// 	SmartDashboard.putNumber("Start Pose X", startPose.getX());
    //     SmartDashboard.putNumber("Start Pose Y", startPose.getY());
    //     SmartDashboard.putNumber("Start Pose Heading", startPose.getRotation().getDegrees());
    
    //     SmartDashboard.putNumber("End Pose X", endPose.getX());
    //     SmartDashboard.putNumber("End Pose Y", endPose.getY());
    //     SmartDashboard.putNumber("End Pose Heading", endPose.getRotation().getDegrees());
        
	// 	// waypoints.add(new Translation2d(endPose.getX() + 1, endPose.getY() + 0.1));
	// 	trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
	// 				waypoints,
    //     			endPose, config);

	// 	Log.writeln("Initial Pose: " + trajectory.getInitialPose());
	// 	Log.writeln("Waypoints:" + waypoints);
	// 	Log.writeln("End Pose:" + endPose);

    //     this.printTrajectory(trajectory);

    //     return trajectory;

	// }

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

	// 	Log.writeln("Trajectory total time: " + trajectory.getTotalTimeSeconds()); 
	// }	

	public Command getAutonomousCommand() {
		return this.chooser.getSelected();
	}
}
