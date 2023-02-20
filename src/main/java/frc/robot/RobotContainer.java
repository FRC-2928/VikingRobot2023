package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.MoveElevatorAndArm;
import frc.robot.commands.POVSelector;
import frc.robot.commands.DrivetrainCommands.BalanceAUX;
import frc.robot.commands.DrivetrainCommands.BalancePID;
import frc.robot.commands.ElevatorCommands.ElevatorGoToHeight;
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

	public Command getAutonomousCommand() {
		return this.chooser.getSelected();
	}
}
