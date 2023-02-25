package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/*
 * When the elevator is lost ( :( ), moves the elevator up until it hits either the home limit switch, or the top limit switch, thus knowing where the elevator is
 */
public class FindHome extends CommandBase {
	private Elevator elevator;
	
	public FindHome(Elevator elevator) {
		this.elevator = elevator;
		this.addRequirements(elevator);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		this.elevator.setPower(ElevatorConstants.defaultPower);
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		// TODO: finish when reaching home or limit switch
		return false;
	}
}
