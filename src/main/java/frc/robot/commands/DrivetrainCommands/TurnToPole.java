package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class TurnToPole extends PIDCommand {
	public TurnToPole(Drivetrain drivetrain) {
		super(
			new PIDController(
				DrivetrainConstants.GainsTurnRetroflective.P, 
				DrivetrainConstants.GainsTurnRetroflective.I, 
				DrivetrainConstants.GainsTurnRetroflective.D
			),
			() -> drivetrain.getTargetHorizontalOffset(),
			0,
			output -> drivetrain.tankDriveVolts(output * 12, -output * 12)
		);

		this.addRequirements(drivetrain);
		this.m_controller.setTolerance(1.5);
	}

	@Override
	public boolean isFinished() {
		return this.m_controller.atSetpoint();
	}
}
