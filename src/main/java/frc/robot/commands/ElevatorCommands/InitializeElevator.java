package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

// When the elevator is lost, moves the elevator up until it hits either the home limit switch, or the top limit switch, thus knowing where the elevator is
public class InitializeElevator extends CommandBase {
	private Elevator elevator;

	public InitializeElevator(Elevator elevator) {
		this.elevator = elevator;
		this.addRequirements(elevator);
	}

	@Override
	public void initialize() {
		this.elevator.motor.overrideLimitSwitchesEnable(true);
		this.elevator.motor.overrideSoftLimitsEnable(false);
	}

	@Override
	public void execute() {
		this.elevator.control(ElevatorConstants.homingPower);
	}

	@Override
	public void end(boolean interrupted) {
		this.elevator.motor.overrideLimitSwitchesEnable(true);
		this.elevator.motor.overrideSoftLimitsEnable(true);
	}

	@Override
	public boolean isFinished() {
		if(this.elevator.limitTopClosed()) {
			this.elevator.halt();
			this.elevator.overrideEncoderPosition(ElevatorConstants.topOffset);
			return true;
		} else if(this.elevator.limitHomeClosed()) {
			this.elevator.halt();
			this.elevator.overrideEncoderPosition(ElevatorConstants.homeOffset);
			return true;
		}
		return false;
	}
}
