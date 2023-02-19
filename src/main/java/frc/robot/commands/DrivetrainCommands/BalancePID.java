package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class BalancePID extends PIDCommand {
	private double time = System.currentTimeMillis();

	/// Whether or not the command should stop when it reaches the setpoint within the tolerance
	public boolean stopAtSetpoint;

	/// The timeout to automatically end the command at
	/// Set to Double.POSITIVE_INFINITY to disable
	public double timeout;
	
	public BalancePID(Drivetrain drivetrain, boolean stopAtSetpoint, double timeout) {
		super(
			new PIDController(
				.4,
				DrivetrainConstants.GainsBalance.I,
				DrivetrainConstants.GainsBalance.D
			),
			() -> drivetrain.readPitch(),
			-15,
			output -> drivetrain.tankDriveVolts(output, output)
		);

		this.m_controller.setTolerance(1.0);
		//this.m_controller.setSetpoint(0.0);
		//this.m_controller.calculate(0.0);

		this.stopAtSetpoint = stopAtSetpoint;
		this.timeout = timeout;

		this.addRequirements(drivetrain);
	}

	/// Construct a manual-style command, which does not stop at setpoint, nor does it timeout.
	public static BalancePID manual(Drivetrain drivetrain) {
		return new BalancePID(drivetrain, false, Double.POSITIVE_INFINITY);
	}

	/// Construct an auto-style command, which automatically stops at setpoint or after `timeout` ms.
	public static BalancePID auto(Drivetrain drivetrain, double timeout) {
		return new BalancePID(drivetrain, true, timeout);
	}

	/// Construct an auto-style command, which automatically stops at setpoint, but does not timeout.
	public static BalancePID auto(Drivetrain drivetrain) {
		return new BalancePID(drivetrain, true, Double.POSITIVE_INFINITY);
	}

	@Override
	public void initialize() {
		this.time = System.currentTimeMillis();
	}

	@Override
	public boolean isFinished() {
		return (this.m_controller.atSetpoint());
	}
}
