package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.commands.ArmCommands.ArmGoToPosition;
import frc.robot.subsystems.*;

public class StashIntake extends SequentialCommandGroup {
	public StashIntake(Elevator elevator, Arm arm) {
		this.addCommands(
			new ElevatorGoToHeight(elevator, ElevatorConstants.highHeight, 1.5),
			new ArmGoToPosition(arm, ArmConstants.inPosition, 1.5)
		);
	}
}
