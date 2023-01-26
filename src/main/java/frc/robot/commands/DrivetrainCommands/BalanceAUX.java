package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class BalanceAUX extends CommandBase {
	private double time = System.currentTimeMillis();

	private Drivetrain drivetrain;

	private PIDController balance = new PIDController(
		DrivetrainConstants.GainsBalance.P,
		DrivetrainConstants.GainsBalance.I,
		DrivetrainConstants.GainsBalance.D
	);

	private PIDController roll = new PIDController(
		DrivetrainConstants.GainsRollBalance.P * 0.25,
		DrivetrainConstants.GainsRollBalance.I,
		DrivetrainConstants.GainsRollBalance.D
	);

	/// Whether or not the command should stop when it reaches the setpoint within the tolerance
	public boolean stopAtSetpoint;

	/// The timeout to automatically end the command at
	/// Set to Double.POSITIVE_INFINITY to disable
	public double timeout;
	
	public BalanceAUX(Drivetrain drivetrain, boolean stopAtSetpoint, double timeout) {
		this.drivetrain = drivetrain;

		this.balance.setTolerance(1.0);
		this.balance.setSetpoint(0.0);
		this.balance.calculate(0.0);

		this.roll.setTolerance(0.3);
		this.roll.setSetpoint(0.0);
		this.roll.calculate(0.0);

		this.stopAtSetpoint = stopAtSetpoint;
		this.timeout = timeout;

		this.addRequirements(drivetrain);
	}

	/// Construct a manual-style command, which does not stop at setpoint, nor does it timeout.
	public static BalanceAUX manual(Drivetrain drivetrain) {
		return new BalanceAUX(drivetrain, false, Double.POSITIVE_INFINITY);
	}

	/// Construct an auto-style command, which automatically stops at setpoint or after `timeout` ms.
	public static BalanceAUX auto(Drivetrain drivetrain, double timeout) {
		return new BalanceAUX(drivetrain, true, timeout);
	}

	/// Construct an auto-style command, which automatically stops at setpoint, but does not timeout.
	public static BalanceAUX auto(Drivetrain drivetrain) {
		return new BalanceAUX(drivetrain, true, Double.POSITIVE_INFINITY);
	}

	@Override
	public void initialize() {
		this.time = System.currentTimeMillis();
	}

	@Override
	public void execute() {
		double balanceVolts = this.balance.calculate(this.drivetrain.readPitch());
		double rollVolts = this.roll.calculate(this.drivetrain.readRoll());
		
		if(this.drivetrain.readPitch() > 0) this.drivetrain.tankDriveVolts(-balanceVolts + rollVolts, -balanceVolts - rollVolts);
		else this.drivetrain.tankDriveVolts(-balanceVolts - rollVolts, -balanceVolts + rollVolts);
	}

	@Override
	public boolean isFinished() {
		return System.currentTimeMillis() > this.time + this.timeout || (this.stopAtSetpoint && this.balance.atSetpoint() && this.roll.atSetpoint());
	}
}
