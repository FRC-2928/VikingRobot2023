package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ArmConstants;
//import frc.robot.Constants.ArmConstants;
//import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.ElevatorConstants;
//import frc.robot.commands.MoveElevatorAndArm;
import frc.robot.commands.POVSelector;
import frc.robot.commands.ArmCommands.ArmGoToPosition;
import frc.robot.commands.DrivetrainCommands.Balance;
import frc.robot.commands.ElevatorCommands.ElevatorGoToHeight;
import frc.robot.commands.ElevatorCommands.GroundIntake;
import frc.robot.commands.ElevatorCommands.InitializeElevator;
import frc.robot.commands.ElevatorCommands.StashIntake;
import frc.robot.commands.IntakeCommands.RunIntake;
//import frc.robot.commands.ElevatorCommands.MoveElevator;
//import frc.robot.commands.ElevatorCommands.ElevatorGoToHeight;
import frc.robot.commands.POVSelector.Tree;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.Arm;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// Mechanism Subsystems
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	public final Transmission transmission = Transmission.instance;
	public final Drivetrain drivetrain = new Drivetrain();

	public final Elevator elevator = new Elevator();
	public final Arm arm = new Arm();
	public final Intake intake = new Intake();

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
					this.driverOI.getMoveSupplier().getAsDouble() * DrivetrainConstants.manualDriveMultiplier,
					this.driverOI.getRotateSupplier().getAsDouble() * DrivetrainConstants.manualDriveMultiplier
				),
				this.drivetrain
			)
		);
		
		/*
		this.drivetrain.setDefaultCommand(
			new RunCommand(
				() -> this.drivetrain.diffDrive.tankDrive(
					this.driverOI.getMoveRSupplier().getAsDouble() * DrivetrainConstants.manualDriveMultiplier,
					this.driverOI.getMoveSupplier().getAsDouble() * DrivetrainConstants.manualDriveMultiplier
				),
				this.drivetrain
			)
		);
		*/

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
		this.elevator.setDefaultCommand(new RunCommand(() -> this.elevator.control(this.operatorOI.getElevatorSupplier().getAsDouble()), this.elevator));
		this.arm.setDefaultCommand(new RunCommand(() -> this.arm.control(this.operatorOI.getArmSupplier().getAsDouble()), this.arm));

		this.operatorOI.getRunIntakeButton().whileTrue(new RunIntake(intake, IntakeConstants.intakePower));
		this.operatorOI.getShootCubeButton().whileTrue(new RunIntake(intake, IntakeConstants.shootCubePower));
		this.operatorOI.getShootConeButton().whileTrue(new RunIntake(intake, IntakeConstants.shootConePower));
		//this.operatorOI.getStopIntakeButton().onTrue(new InstantCommand(() -> this.intake.setOutput(0)));

		this.operatorOI.getInitializeElevatorButton().onTrue(new InitializeElevator(this.elevator));
		
		this.operatorOI.getArmHigh().onTrue(new ArmGoToPosition(arm, ArmConstants.highPosition));
		this.operatorOI.getArmHigh().onTrue(new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight));
		this.operatorOI.getArmMid().onTrue(new ArmGoToPosition(arm, ArmConstants.midPosition));
		this.operatorOI.getArmMid().onTrue(new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight));
		this.operatorOI.getArmGroundCube().onTrue(new GroundIntake(elevator, arm, GamePiece.Cube));
		this.operatorOI.getArmGroundCone().onTrue(new GroundIntake(elevator, arm, GamePiece.Cone));
		this.operatorOI.getArmIn().onTrue(new StashIntake(elevator, arm));

		this.operatorOI.getElevatorDown().onTrue(new ElevatorGoToHeight(elevator, ElevatorConstants.stashHeight));
		this.operatorOI.getElevatorUp().onTrue(new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight));
	}

	private void configureAutoChooser() {
		this.autonomousChooser = AutonomousRoutines.createAutonomousChooser(this.drivetrain);
		SmartDashboard.putData("Autonomous Routine", this.autonomousChooser);
	}

	public Command getAutonomousCommand() {
		return this.autonomousChooser.getSelected();
	}
}
