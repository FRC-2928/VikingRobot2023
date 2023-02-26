package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.Constants.ArmConstants;
//import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
//import frc.robot.Constants.ElevatorConstants;
//import frc.robot.commands.MoveElevatorAndArm;
import frc.robot.commands.POVSelector;
import frc.robot.commands.DrivetrainCommands.Balance;
import frc.robot.commands.ElevatorCommands.InitializeElevator;
//import frc.robot.commands.ElevatorCommands.MoveElevator;
//import frc.robot.commands.ElevatorCommands.ElevatorGoToHeight;
import frc.robot.commands.POVSelector.Tree;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.AutonomousRoutines;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;

// Mechanism Subsystems
import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	public final Transmission transmission = Transmission.instance;
	public final Drivetrain drivetrain = new Drivetrain();
	// public final Intake intake = new Intake();
	public final Elevator elevator = new Elevator();
	public final Arm arm = new Arm();

	private final XboxController driverController = new XboxController(0);
	private final XboxController operatorController = new XboxController(1);
	public final DriverOI driverOI = new DriverOI(this.driverController);
	public final OperatorOI operatorOI = new OperatorOI(this.operatorController);

	private SendableChooser<Command> autonomousChooser = new SendableChooser<>();
	//private ShuffleboardTab tab = Shuffleboard.getTab("ElevatorArm");

	public RobotContainer() {
		this.configureAutoChooser();

		this.configureDriverControls();
		this.configureOperatorControls();
	}

	private void configureDriverControls() {
		this.drivetrain.setDefaultCommand(
			new RunCommand(
				() -> this.drivetrain.diffDrive.arcadeDrive(
					this.driverOI.getMoveSupplier().getAsDouble() * DrivetrainConstants.arcadeDriveMultiplier,
					this.driverOI.getRotateSupplier().getAsDouble() * DrivetrainConstants.arcadeDriveMultiplier
				),
				this.drivetrain
			)
		);

		// Configure gear shifting
		this.driverOI.getShiftLowButton().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.driverOI.getShiftHighButton().onTrue(new InstantCommand(this.transmission::setHigh, this.transmission));

		this.driverOI.getBalanceAuxButton().whileTrue(Balance.manual(this.drivetrain));

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
					.schedule(TrajectoryRunner.generateRamseteCommand(this.drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(this.drivetrain, (Direction)dir)));
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

	private void configureOperatorControls() {
		// default command should run only in absence of other commands - shouldn't be a problem for these to be default even though they're backup. (CHECK THOUGH)
		// why is this a good idea -nova
		this.elevator.setDefaultCommand(new RunCommand(() -> this.elevator.control(operatorOI.getElevatorSupplier().getAsDouble()), this.elevator));
		this.arm.setDefaultCommand(new RunCommand(() -> this.arm.setPower(operatorOI.getArmSupplier().getAsDouble()), this.arm));

		// this.operatorOI.getRunIntakeButton().onTrue(new InstantCommand(() -> intake.setOutput(IntakeConstants.intakePower)));
		// this.operatorOI.getShootIntakeButton().onTrue(new InstantCommand(() -> intake.setOutput(IntakeConstants.shootPower)));
		// this.operatorOI.getStopIntakeButton().onTrue(new InstantCommand(() -> intake.setOutput(0)));

		this.operatorOI.getInitializeElevatorButton().onTrue(new InitializeElevator(elevator));
		// this.operatorOI.getHigh().onTrue(new MoveElevatorAndArm(elevator, arm, ElevatorConstants.highHeight, ArmConstants.highPosition));
		// this.operatorOI.getMid().onTrue(new MoveElevatorAndArm(elevator, arm, ElevatorConstants.highHeight, ArmConstants.midPosition));
		// this.operatorOI.getLow().onTrue(new MoveElevatorAndArm(elevator, arm, ElevatorConstants.lowHeight, ArmConstants.lowPosition));
	}

	private void configureAutoChooser() {
		this.autonomousChooser = AutonomousRoutines.createAutonomousChooser(this.drivetrain);
		SmartDashboard.putData("Autonomous Routine", this.autonomousChooser);
	}

	public Command getAutonomousCommand() {
		return this.autonomousChooser.getSelected();
	}
}
