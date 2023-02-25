package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ArmGoToPosition;
import frc.robot.commands.ElevatorCommands.ElevatorGoToHeight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class MoveElevatorAndArm extends SequentialCommandGroup {
	public MoveElevatorAndArm(Elevator elevator, Arm arm, double elevatorGoal, double armGoal) {
		//if going down
		if (elevator.getEncoderTicks() > elevatorGoal) {
			this.addCommands(
				new ArmGoToPosition(arm, armGoal),
				new ElevatorGoToHeight(elevator, elevatorGoal)
			);
		} else {
			this.addCommands(
				new ElevatorGoToHeight(elevator, elevatorGoal),
				new ArmGoToPosition(arm, armGoal)
			);
		}
	}
}
