package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ArmCommands.ArmGoToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class GroundIntake extends SequentialCommandGroup {
	public GroundIntake(Elevator elevator, Arm arm) {
		this.addCommands(new ArmGoToPosition(arm, ArmConstants.lowPosition),
						new ElevatorGoToHeight(elevator, ElevatorConstants.lowHeight));
	}
}
