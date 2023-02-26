package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class MoveElevatorAndArm extends ParallelCommandGroup {
	public MoveElevatorAndArm(Elevator elevator, Arm arm, double elevatorPosition, double armPosition) {
		this.addCommands();
	}
}
