// package frc.robot.commands.DrivetrainCommands;
package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Drivetrain;

public class ApproachTag extends CommandBase {
	private double time = System.currentTimeMillis();

	private Drivetrain drivetrain;

	private PIDController approach = new PIDController(
		DrivetrainConstants.GainsApproach.P,
		DrivetrainConstants.GainsApproach.I,
		DrivetrainConstants.GainsApproach.D
	);

	private PIDController turn = new PIDController(
		DrivetrainConstants.GainsApproachTurn.P,
		DrivetrainConstants.GainsApproachTurn.I,
		DrivetrainConstants.GainsApproachTurn.D
	);

	/// Whether or not the command should stop when it reaches the setpoint within the tolerance
	public boolean stopAtSetpoint;

	/// The timeout to automatically end the command at
	/// Set to Double.POSITIVE_INFINITY to disable
	public double timeout;
	
	public ApproachTag(Drivetrain drivetrain, boolean stopAtSetpoint, double timeout) {
		this.drivetrain = drivetrain;

		this.approach.setTolerance(0);
		this.approach.setSetpoint(LimelightConstants.tagGoalY);
		this.approach.calculate(0.0);

		this.turn.setTolerance(0.1);
		this.turn.setSetpoint(0.0);
		this.turn.calculate(0.0);

		this.stopAtSetpoint = stopAtSetpoint;
		this.timeout = timeout;

		this.addRequirements(drivetrain);
	}

	/// Construct a manual-style command, which does not stop at setpoint, nor does it timeout.
	public static ApproachTag manual(Drivetrain drivetrain) {
		return new ApproachTag(drivetrain, false, Double.POSITIVE_INFINITY);
	}

	/// Construct an auto-style command, which automatically stops at setpoint or after `timeout` ms.
	public static ApproachTag auto(Drivetrain drivetrain, double timeout) {
		return new ApproachTag(drivetrain, true, timeout);
	}

	/// Construct an auto-style command, which automatically stops at setpoint, but does not timeout.
	public static ApproachTag auto(Drivetrain drivetrain) {
		return new ApproachTag(drivetrain, true, Double.POSITIVE_INFINITY);
	}

	@Override
	public void initialize() {
		this.time = System.currentTimeMillis();
	}

	@Override
	public void execute() {
		double approachVolts = this.approach.calculate(this.drivetrain.getTargetVerticalOffset());
		double turnVolts = this.turn.calculate(this.drivetrain.getTargetHorizontalOffset());
		
		this.drivetrain.tankDriveVolts(approachVolts + turnVolts, approachVolts - turnVolts);
	}

	@Override
	public boolean isFinished() {
		return System.currentTimeMillis() > this.time + this.timeout || (this.stopAtSetpoint && this.approach.atSetpoint() && this.turn.atSetpoint());
	}
}
