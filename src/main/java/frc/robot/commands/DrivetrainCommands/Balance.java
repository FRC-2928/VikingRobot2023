package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {
	private double time = System.currentTimeMillis();

	private final Drivetrain drivetrain;

	private final PIDController balance = new PIDController(
		DrivetrainConstants.GainsBalance.P,
		DrivetrainConstants.GainsBalance.I,
		DrivetrainConstants.GainsBalance.D
	);

	private final PIDController align = new PIDController(
		DrivetrainConstants.GainsAlignBalance.P * 0.25,
		DrivetrainConstants.GainsAlignBalance.I,
		DrivetrainConstants.GainsAlignBalance.D
	);

	/// Whether or not the command should stop when it reaches the setpoint within the tolerance
	public final boolean stopAtSetpoint;

	/// The timeout to automatically end the command at
	/// Set to Double.POSITIVE_INFINITY to disable
	public final double timeout;

	public Balance(final Drivetrain drivetrain, final boolean stopAtSetpoint, final double timeout) {
		this.drivetrain = drivetrain;

		this.balance.setTolerance(8.0);
		this.balance.setSetpoint(0.0);
		this.balance.calculate(0.0);

		this.align.setTolerance(0.3);
		this.align.setSetpoint(0.0);
		this.align.calculate(0.0);

		this.stopAtSetpoint = stopAtSetpoint;
		this.timeout = timeout;

		this.addRequirements(drivetrain);
	}

	/// Construct a manual-style command, which does not stop at setpoint, nor does it timeout.
	public static Balance manual(final Drivetrain drivetrain) {
		return new Balance(drivetrain, false, Double.POSITIVE_INFINITY);
	}

	/// Construct an auto-style command, which automatically stops at setpoint or after `timeout` ms.
	public static Balance auto(final Drivetrain drivetrain, final double timeout) {
		return new Balance(drivetrain, true, timeout);
	}

	/// Construct an auto-style command, which automatically stops at setpoint, but does not timeout.
	public static Balance auto(final Drivetrain drivetrain) {
		return new Balance(drivetrain, true, Double.POSITIVE_INFINITY);
	}

	/// Construct an auto-style command, which does not stop at setpoint, but does timeout.
	public static Balance timed(final Drivetrain drivetrain, final double timeout) {
		return new Balance(drivetrain, false, timeout);
	}

	@Override
	public void initialize() {
		this.time = System.currentTimeMillis();
	}

	@Override
	public void execute() {
		final double balanceVolts = this.balance.calculate(this.drivetrain.readPitch());
		final double alignVolts = this.align.calculate(this.drivetrain.readRoll());

		if(this.drivetrain.readPitch() > 0) this.drivetrain.tankDriveVolts(-balanceVolts + alignVolts, -balanceVolts - alignVolts);
		else this.drivetrain.tankDriveVolts(-balanceVolts - alignVolts, -balanceVolts + alignVolts);

        this.drivetrain.setBrakeMode();
	}

	@Override
	public void end(final boolean interrupted) {
		if(DriverStation.getMatchTime() > 15) this.drivetrain.setCoastMode();
	}

	@Override
	public boolean isFinished() {
		return System.currentTimeMillis() > this.time + this.timeout || (this.stopAtSetpoint && this.balance.atSetpoint() && this.align.atSetpoint());
	}
}
