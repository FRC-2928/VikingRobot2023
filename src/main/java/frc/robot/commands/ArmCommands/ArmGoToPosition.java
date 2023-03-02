package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmGoToPosition extends PIDCommand {
	public ArmGoToPosition(Arm arm, double goalPosition) {
		super(
			new PIDController(ArmConstants.armGains.P, ArmConstants.armGains.I, ArmConstants.armGains.D),
			() -> arm.getPosition(),
			goalPosition,
			output -> arm.control(-output)
		);
		this.addRequirements(arm);
		this.m_controller.setTolerance(2);
	}

	@Override
	public boolean isFinished() {
		return getController().atSetpoint();
	}
}
