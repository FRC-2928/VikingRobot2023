package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class BalanceRollPID extends PIDCommand {
	private double time = System.currentTimeMillis();

	/// Whether or not the command should stop when it reaches the setpoint within the tolerance
	public boolean stopAtSetpoint;

	/// The timeout to automatically end the command at
	/// Set to Double.POSITIVE_INFINITY to disable
	public double timeout;

	public BalanceRollPID(Drivetrain drivetrain, boolean stopAtSetpoint, double timeout) {
		super(
			new PIDController(
				DrivetrainConstants.GainsRollBalance.P,
				DrivetrainConstants.GainsRollBalance.I,
				DrivetrainConstants.GainsRollBalance.D
			),
			() -> drivetrain.readGyro()[1],
			0,
			output -> {
				if (drivetrain.readPitch() > 0)  drivetrain.tankDriveVolts(output, -output);
				else drivetrain.tankDriveVolts(-output, output);
			}
		);
		
		this.m_controller.setTolerance(0.3);
		this.m_controller.setSetpoint(0.0);
		this.m_controller.calculate(0.0);

		this.stopAtSetpoint = stopAtSetpoint;
		this.timeout = timeout;

		this.addRequirements(drivetrain);
	}

	/// Construct a manual-style command, which does not stop at setpoint, nor does it timeout.
	public static BalanceRollPID manual(Drivetrain drivetrain) {
		return new BalanceRollPID(drivetrain, false, Double.POSITIVE_INFINITY);
	}

	/// Construct an auto-style command, which automatically stops at setpoint or after `timeout` ms.
	public static BalanceRollPID auto(Drivetrain drivetrain, double timeout) {
		return new BalanceRollPID(drivetrain, true, timeout);
	}

	/// Construct an auto-style command, which automatically stops at setpoint, but does not timeout.
	public static BalanceRollPID auto(Drivetrain drivetrain) {
		return new BalanceRollPID(drivetrain, true, Double.POSITIVE_INFINITY);
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
