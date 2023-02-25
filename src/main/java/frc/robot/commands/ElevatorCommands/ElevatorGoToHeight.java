package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorGoToHeight extends PIDCommand {
	public ElevatorGoToHeight(Elevator elevator, double goalHeight) {
		super(
			new PIDController(ElevatorConstants.elevatorGains.P, ElevatorConstants.elevatorGains.I, ElevatorConstants.elevatorGains.D),
			() -> elevator.getEncoderTicks(),
			goalHeight,
			output -> elevator.setPower(output)
		);

		this.m_controller.setTolerance(30);
		this.addRequirements(elevator);
	}

	@Override
	public boolean isFinished() {
		return this.m_controller.atSetpoint();
	}
}
