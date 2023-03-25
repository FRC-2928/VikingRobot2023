package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.commands.ArmCommands.ArmGoToPosition;
import frc.robot.subsystems.*;

public class GroundIntake extends SequentialCommandGroup {
	public GroundIntake(Elevator elevator, Arm arm, GamePiece gpt) {
		this.addCommands(
			new ArmGoToPosition(arm, (gpt == GamePiece.Cone) ? ArmConstants.lowPositionCone : ArmConstants.lowPositionCube),
			new ElevatorGoToHeight(elevator, ElevatorConstants.lowHeight)
		);
	}
}
