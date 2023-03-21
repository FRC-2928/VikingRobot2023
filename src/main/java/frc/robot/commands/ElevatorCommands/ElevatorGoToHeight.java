package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorGoToHeight extends PIDCommand {
	private Elevator elevator;

	public ElevatorGoToHeight(Elevator elevator, double goalHeight) {
		this(elevator, goalHeight, 1);
	}

	public ElevatorGoToHeight(Elevator elevator, double goalHeight, double speedFactor) {
		super(
			new PIDController(ElevatorConstants.elevatorGains.P * speedFactor, ElevatorConstants.elevatorGains.I, ElevatorConstants.elevatorGains.D),
			() -> elevator.getPosition(),
			goalHeight - ElevatorConstants.averageLockIntervalTicks,
			output -> elevator.control(output)
		);

		this.elevator = elevator;
		
		this.m_controller.setTolerance(1000);
		this.addRequirements(elevator);
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
