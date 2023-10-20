package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.*;
import frc.robot.commands.DrivetrainCommands.*;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.ArmCommands.*;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.commands.POVSelector;
import frc.robot.oi.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.*;

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

	public final LimelightFX fx = new LimelightFX(SerialPort.Port.kUSB);

	public final Mechanism2d mech;
	public final MechanismRoot2d mechRoot;
	public final MechanismLigament2d mechElevator;
	public final MechanismLigament2d mechElevatorExtension;
	public final MechanismLigament2d mechArm;

	private final XboxController driverController = new XboxController(0);
	private final XboxController operatorController = new XboxController(1);
	public final DriverOI driverOI = new DriverOI(this.driverController);
	public final OperatorOI operatorOI = new OperatorOI(this.operatorController);

	private final SendableChooser<Command> autonomousChooser = AutonomousRoutines.createAutonomousChooser(this.drivetrain, this.elevator, this.arm, this.intake);

	public RobotContainer() {
		SmartDashboard.putData("Autonomous Routine", this.autonomousChooser);

		this.configureDriverControls();
		this.configureOperatorControls();

		this.mech = new Mechanism2d(10, 10, new Color8Bit(0, 0, 0));
		this.mechRoot = this.mech.getRoot("Root", 0, 0);
		this.mechElevator = this.mechRoot.append(new MechanismLigament2d("Elevator", GlassMechanismConstants.elevator.length, 30, 0, new Color8Bit(255, 0, 0)));
		this.mechElevatorExtension = this.mechElevator.append(new MechanismLigament2d("ElevatorExtension", GlassMechanismConstants.elevator.length, 30, 0, new Color8Bit(255, 0, 0)));
		this.mechArm = this.mechElevatorExtension.append(new MechanismLigament2d("Arm", GlassMechanismConstants.elevator.length, 30, 0, new Color8Bit(255, 0, 0)));

		SmartDashboard.putData(this.mech);
	}

	private void configureDriverControls() {
		this.drivetrain.setDefaultCommand(
			new RunCommand(
				() -> {
					final double clampTo = this.arm.armIsOut() ? 0.6 : 1;

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
		this.driverOI.getShiftButton().whileTrue(new Shift(this.transmission, Transmission.GearState.HIGH));

		// this.driverOI.getSetBrakeButton().onTrue(new InstantCommand(this.drivetrain::setBrakeMode, this.drivetrain));
		// this.driverOI.getSetCoastButton().onTrue(new InstantCommand(this.drivetrain::setCoastMode, this.drivetrain));
		this.driverOI.getBalanceButton().onTrue(new InstantCommand(this.drivetrain::setBrakeMode, this.drivetrain));

		this.driverOI.getBalanceButton().whileTrue(Balance.manual(this.drivetrain));

		this.driverOI.getCenterOnPoleButton().onTrue(new TurnToPole(this.drivetrain));

		this.driverOI.getApproachTagButton().toggleOnTrue(new POVSelector(
			this.driverOI,
			null,
			(dir, __) -> {
				CommandScheduler
					.getInstance()
					.schedule(TrajectoryRunner.generateRamseteCommand(this.drivetrain, () -> TrajectoryRunner.generateLocalTrajectory(this.drivetrain, (TrajectoryRunner.Direction)dir)));
			},
			new POVSelector.Tree(
				"Select tag offset",
				new POVSelector.Tree("Center", TrajectoryRunner.Direction.Center),
				new POVSelector.Tree("Left", TrajectoryRunner.Direction.Left),
				new POVSelector.Tree(),
				new POVSelector.Tree("Right", TrajectoryRunner.Direction.Right)
			)
		));

		this.driverOI.getRunIntakeButton().whileTrue(new RunIntake(this.intake, IntakeConstants.intakePower));

		this.driverOI.getHaltButton().onTrue(new InstantCommand(() -> {
			CommandScheduler.getInstance().cancelAll();
			Log.warning("[HALT - DRIVER]");
			this.drivetrain.halt();
			this.elevator.halt();
			this.arm.halt();
			this.intake.setOutput(0);
		}));
	}

	private void configureOperatorControls() {
		this.elevator.setDefaultCommand(new RunCommand(() -> this.elevator.control(this.operatorOI.getElevatorSupplier().getAsDouble()), this.elevator));
		this.arm.setDefaultCommand(new RunCommand(() -> this.arm.control(this.operatorOI.getArmSupplier().getAsDouble()), this.arm));

		this.operatorOI.getIntakeButton().whileTrue(new RunIntake(this.intake, IntakeConstants.intakePower));
		this.operatorOI.getShootCubeButton().whileTrue(new RunIntake(this.intake, IntakeConstants.shootCubePower));
		this.operatorOI.getShootConeButton().whileTrue(new RunIntake(this.intake, IntakeConstants.shootConePower));

		this.operatorOI.getInitializeElevatorButton().onTrue(new InitializeElevator(this.elevator));

		this.operatorOI.getArmHigh().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.operatorOI.getArmHigh().onTrue(new ArmGoToPosition(this.arm, ArmConstants.highPosition));
		this.operatorOI.getArmHigh().onTrue(new ElevatorGoToHeight(this.elevator, ElevatorConstants.highHeight));
		this.operatorOI.getArmMid().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.operatorOI.getArmMid().onTrue(new ArmGoToPosition(this.arm, ArmConstants.midPosition));
		this.operatorOI.getArmMid().onTrue(new ElevatorGoToHeight(this.elevator, ElevatorConstants.highHeight));
		this.operatorOI.getArmGroundCube().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.operatorOI.getArmGroundCube().onTrue(new GroundIntake(this.elevator, this.arm, GamePiece.Cube));
		this.operatorOI.getArmGroundCone().onTrue(new InstantCommand(this.transmission::setLow, this.transmission));
		this.operatorOI.getArmGroundCone().onTrue(new GroundIntake(this.elevator, this.arm, GamePiece.Cone));
		this.operatorOI.getArmStash().onTrue(
            new StashIntake(this.elevator, this.arm)
                .alongWith(new InstantCommand(() -> this.displayImage("heart"), this.fx).unless(DriverStation::isAutonomousEnabled))
        );

		this.operatorOI.getArmSubstationCone().onTrue(
            new ElevatorGoToHeight(this.elevator, ElevatorConstants.highHeight)
                .andThen(new ArmGoToPosition(this.arm, ArmConstants.doubleSubstationCone))
                .alongWith(new InstantCommand(() -> this.displayImage("cone"), this.fx).unless(DriverStation::isAutonomousEnabled))
                .alongWith(new InstantCommand(this.transmission::setLow, this.transmission))
		);
		this.operatorOI.getArmSubstationCube().onTrue(
            new ElevatorGoToHeight(this.elevator, ElevatorConstants.highHeight)
                .andThen(new ArmGoToPosition(this.arm, ArmConstants.doubleSubstationCube))
                .alongWith(new InstantCommand(() -> this.displayImage("cube3d"), this.fx).unless(DriverStation::isAutonomousEnabled))
                .alongWith(new InstantCommand(this.transmission::setLow, this.transmission))
		);

		this.operatorOI.getHaltButton().onTrue(new InstantCommand(() -> {
			CommandScheduler.getInstance().cancelAll();
			Log.warning("[HALT - OPERATOR]");
			this.drivetrain.halt();
			this.elevator.halt();
			this.arm.halt();
			this.intake.setOutput(0);
		}));
	}

    public void displayImage(final String image) {
        this.fx.image(LimelightFXConstants.image(image));
    }

	public Command getAutonomousCommand() {
		return this.autonomousChooser.getSelected();
	}
}
