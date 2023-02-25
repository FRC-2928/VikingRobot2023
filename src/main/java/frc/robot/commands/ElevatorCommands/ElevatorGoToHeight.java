package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorGoToHeight extends PIDCommand {
	private Elevator elevator;

	public ElevatorGoToHeight(Elevator elevator, double goalHeight) {
		super(
			new PIDController(ElevatorConstants.elevatorGains.P, ElevatorConstants.elevatorGains.I, ElevatorConstants.elevatorGains.D),
			() -> elevator.getEncoderPosition(),
			goalHeight,
			output -> elevator.control(output)
		);

		this.m_controller.setTolerance(30);
		this.addRequirements(elevator);
		this.elevator = elevator;
	}

	@Override
	public void initialize() {}

	@Override
	public void end(boolean interrupted) {
		this.elevator.control(0.0);
	}

	@Override
	public boolean isFinished() {
		return this.m_controller.atSetpoint();
	}
}
