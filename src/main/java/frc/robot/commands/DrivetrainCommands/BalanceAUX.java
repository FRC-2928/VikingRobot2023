package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class BalanceAUX extends PIDCommand {
	private double time = System.currentTimeMillis();

	/// Whether or not the command should stop when it reaches the setpoint within the tolerance
	public boolean stopAtSetpoint;

	/// The timeout to automatically end the command at
	/// Set to Double.POSITIVE_INFINITY to disable
	public double timeout;
	
	public BalanceAUX(Drivetrain drivetrain, boolean stopAtSetpoint, double timeout) {
		super(
			new PIDController(
				DrivetrainConstants.GainsBalance.P,
				DrivetrainConstants.GainsBalance.I,
				DrivetrainConstants.GainsBalance.D
			),
			() -> drivetrain.readPitch(),
			0,
			output -> drivetrain.BalanceRollPitch(output)
		);

		this.m_controller.setTolerance(1.0);
		this.m_controller.setSetpoint(0.0);
		this.m_controller.calculate(0.0);

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
	public boolean isFinished() {
		return System.currentTimeMillis() > this.time + this.timeout || (this.stopAtSetpoint && this.m_controller.atSetpoint());
	}
}
