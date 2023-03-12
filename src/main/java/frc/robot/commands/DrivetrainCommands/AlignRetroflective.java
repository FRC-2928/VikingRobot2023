package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class AlignRetroflective extends CommandBase {
	private double time = System.currentTimeMillis();

	private Drivetrain drivetrain;

	private PIDController go = new PIDController(
		DrivetrainConstants.GainsGoRetroflective.P,
		DrivetrainConstants.GainsGoRetroflective.I,
		DrivetrainConstants.GainsGoRetroflective.D
	);

	private PIDController align = new PIDController(
		DrivetrainConstants.GainsTurnRetroflective.P * 0.25,
		DrivetrainConstants.GainsTurnRetroflective.I,
		DrivetrainConstants.GainsTurnRetroflective.D
	);

	/// Whether or not the command should stop when it reaches the setpoint within the tolerance
	public boolean stopAtSetpoint;

	/// The timeout to automatically end the command at
	/// Set to Double.POSITIVE_INFINITY to disable
	public double timeout;

	public AlignRetroflective(Drivetrain drivetrain, boolean stopAtSetpoint, double timeout) {
		this.drivetrain = drivetrain;

		this.go.setTolerance(8.0);
		this.go.setSetpoint(0.0);
		this.go.calculate(0.0);

		this.align.setTolerance(0.3);
		this.align.setSetpoint(0.0);
		this.align.calculate(0.0);

		this.stopAtSetpoint = stopAtSetpoint;
		this.timeout = timeout;

		this.addRequirements(drivetrain);
	}

	/// Construct a manual-style command, which does not stop at setpoint, nor does it timeout.
	public static Balance manual(Drivetrain drivetrain) {
		return new Balance(drivetrain, false, Double.POSITIVE_INFINITY);
	}

	/// Construct an auto-style command, which automatically stops at setpoint or after `timeout` ms.
	public static Balance auto(Drivetrain drivetrain, double timeout) {
		return new Balance(drivetrain, true, timeout);
	}

	/// Construct an auto-style command, which automatically stops at setpoint, but does not timeout.
	public static Balance auto(Drivetrain drivetrain) {
		return new Balance(drivetrain, true, Double.POSITIVE_INFINITY);
	}

	/// Construct an auto-style command, which does not stop at setpoint, but does timeout.
	public static Balance timed(Drivetrain drivetrain, double timeout) {
		return new Balance(drivetrain, false, timeout);
	}

	@Override
	public void initialize() {
		this.time = System.currentTimeMillis();
		
		this.drivetrain.brakeOverride = true;
	}

	@Override
	public void execute() {
		double goVolts = this.go.calculate(this.drivetrain.getBottomLimelightTargetVerticalOffset());
		double alignVolts = this.align.calculate(this.drivetrain.getBottomLimelightTargetHorizontalOffset());

		this.drivetrain.tankDriveVolts(goVolts + alignVolts, goVolts - alignVolts);
		
	}

	@Override
	public void end(boolean interrupted) {
		this.drivetrain.brakeOverride = false;
	}

	@Override
	public boolean isFinished() {
		return System.currentTimeMillis() > this.time + this.timeout || (this.stopAtSetpoint && this.go.atSetpoint() && this.align.atSetpoint());
	}
}
