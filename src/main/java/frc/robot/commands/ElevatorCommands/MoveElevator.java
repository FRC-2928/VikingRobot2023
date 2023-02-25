package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends CommandBase {
	private Elevator elevator;
	private double speed;

	public MoveElevator(Elevator elevator, double speed) {
		addRequirements(elevator);
		this.elevator = elevator;
		this.speed = speed;
	}

	@Override
	public void initialize() {
		this.elevator.control(speed);
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		this.elevator.control(0.0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
