package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ArmCommands.ArmGoToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class StashIntake extends SequentialCommandGroup {
	public StashIntake(Elevator elevator, Arm arm) {
		this.addCommands(
			new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight),
			new ArmGoToPosition(arm, ArmConstants.inPosition)
		);
	}
}
