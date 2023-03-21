package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.commands.DrivetrainCommands.Balance;
import frc.robot.commands.DrivetrainCommands.Shift;
import frc.robot.commands.DrivetrainCommands.TurnToPole;
import frc.robot.commands.ElevatorCommands.ElevatorGoToHeight;
import frc.robot.commands.ElevatorCommands.GroundIntake;
import frc.robot.commands.ElevatorCommands.InitializeElevator;
import frc.robot.commands.ElevatorCommands.StashIntake;
import frc.robot.commands.ArmCommands.ArmGoToPosition;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.POVSelector;
import frc.robot.commands.POVSelector.Tree;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.Log;
import frc.robot.subsystems.Transmission;
import frc.robot.subsystems.Transmission.GearState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TrajectoryRunner;
import frc.robot.subsystems.TrajectoryRunner.Direction;
import frc.robot.subsystems.LimelightFX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	public final Transmission transmission = new Transmission();
	public final Drivetrain drivetrain = new Drivetrain();

	public final Elevator elevator = new Elevator();
	public final Arm arm = new Arm();
	public final Intake intake = new Intake();

	public final LimelightFX limelightFX = new LimelightFX(SerialPort.Port.kUSB);

	private final XboxController driverController = new XboxController(0);
	private final XboxController operatorController = new XboxController(1);
	public final DriverOI driverOI = new DriverOI(this.driverController);
	public final OperatorOI operatorOI = new OperatorOI(this.operatorController);

	private SendableChooser<Command> autonomousChooser = AutonomousRoutines.createAutonomousChooser(this.drivetrain, this.elevator, this.arm, this.intake);

	public RobotContainer() {
		SmartDashboard.putData("Autonomous Routine", this.autonomousChooser);

		this.configureDriverControls();
		this.configureOperatorControls();
	}

	private void configureDriverControls() {
		this.drivetrain.setDefaultCommand(
			new RunCommand(
				() -> {
					double clampTo = this.arm.armIsOut() ? 0.6 : 1;

					this.drivetrain.diffDrive.arcadeDrive(
						Math.min(this.driverOI.getMoveSupplier().getAsDouble() * this.driverOI.getReductFactor() * DrivetrainConstants.manualDriveMultiplier, clampTo),
						Math.min(this.driverOI.getRotateSupplier().getAsDouble() * this.driverOI.getReductFactorRotation() * DrivetrainConstants.manualTurnMultiplier, clampTo)
					);
				},
				this.drivetrain
			)
		);

		// Configure gear shifting
		this.driverOI.getShiftLowButton().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.driverOI.getShiftHighButton().onTrue(new InstantCommand(this.transmission::setHigh, this.transmission));
		this.driverOI.getShiftButton().whileTrue(new Shift(this.transmission, GearState.HIGH));

		this.driverOI.getSetBrakeButton().onTrue(new InstantCommand(this.drivetrain::setBrakeMode, this.drivetrain));
		this.driverOI.getSetCoastButton().onTrue(new InstantCommand(this.drivetrain::setCoastMode, this.drivetrain));
		this.driverOI.getBalanceButton().onTrue(new InstantCommand(this.drivetrain::setBrakeMode, this.drivetrain));

		this.driverOI.getBalanceButton().whileTrue(Balance.manual(this.drivetrain));

		this.driverOI.getCenterOnPoleButton().onTrue(new TurnToPole(drivetrain));

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
				new Tree("Left", Direction.Left),
				new Tree(),
				new Tree("Right", Direction.Right)
			)
		));

		this.driverOI.getRunIntakeButton().whileTrue(new RunIntake(intake, IntakeConstants.intakePower));

		this.driverOI.getHaltButton().onTrue(new InstantCommand(() -> {
			CommandScheduler.getInstance().cancelAll();
			Log.writeln("[HALT - DRIVER]");
			this.drivetrain.halt();
			this.elevator.halt();
			this.arm.halt();
			this.intake.setOutput(0);
		}));
	}

	private void configureOperatorControls() {
		this.elevator.setDefaultCommand(new RunCommand(() -> this.elevator.control(this.operatorOI.getElevatorSupplier().getAsDouble()), this.elevator));
		this.arm.setDefaultCommand(new RunCommand(() -> this.arm.control(this.operatorOI.getArmSupplier().getAsDouble()), this.arm));

		this.operatorOI.getIntakeButton().whileTrue(new RunIntake(intake, IntakeConstants.intakePower));
		this.operatorOI.getShootCubeButton().whileTrue(new RunIntake(intake, IntakeConstants.shootCubePower));
		this.operatorOI.getShootConeButton().whileTrue(new RunIntake(intake, IntakeConstants.shootConePower));
		//this.operatorOI.getStopIntakeButton().onTrue(new InstantCommand(() -> this.intake.setOutput(0)));

		this.operatorOI.getInitializeElevatorButton().onTrue(new InitializeElevator(this.elevator));

		this.operatorOI.getArmHigh().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.operatorOI.getArmHigh().onTrue(new ArmGoToPosition(arm, ArmConstants.highPosition));
		this.operatorOI.getArmHigh().onTrue(new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight));
		this.operatorOI.getArmMid().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.operatorOI.getArmMid().onTrue(new ArmGoToPosition(arm, ArmConstants.midPosition));
		this.operatorOI.getArmMid().onTrue(new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight));
		this.operatorOI.getArmGroundCube().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.operatorOI.getArmGroundCube().onTrue(new GroundIntake(elevator, arm, GamePiece.Cube));
		this.operatorOI.getArmGroundCone().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.operatorOI.getArmGroundCone().onTrue(new GroundIntake(elevator, arm, GamePiece.Cone));
		this.operatorOI.getArmStash().onTrue(new StashIntake(elevator, arm));

		this.operatorOI.getArmSubstationCone().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.operatorOI.getArmSubstationCone().onTrue(
			new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight)
				.andThen(new ArmGoToPosition(arm, ArmConstants.doubleSubstationCone))
		);
		this.operatorOI.getArmSubstationCube().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.operatorOI.getArmSubstationCube().onTrue(
			new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight)
				.andThen(new ArmGoToPosition(arm, ArmConstants.doubleSubstationCube))
		);

		this.operatorOI.getHaltButton().onTrue(new InstantCommand(() -> {
			CommandScheduler.getInstance().cancelAll();
			Log.writeln("[HALT - OPERATOR]");
			this.drivetrain.halt();
			this.elevator.halt();
			this.arm.halt();
			this.intake.setOutput(0);
		}));
	}

	/**
	 * runs when teleop initializes
	 */
	public void onTeleopInit(){
		this.drivetrain.setCoastMode();
	}

	public Command getAutonomousCommand() {
		return this.autonomousChooser.getSelected();
	}
}
