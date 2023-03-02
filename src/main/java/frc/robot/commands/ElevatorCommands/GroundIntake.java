package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.commands.ArmCommands.ArmGoToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class GroundIntake extends SequentialCommandGroup {
	public GroundIntake(Elevator elevator, Arm arm, GamePiece gpt) {
		this.addCommands(
			new ArmGoToPosition(arm, (gpt == GamePiece.Cone) ? ArmConstants.lowPositionCone : ArmConstants.lowPositionCube),
			new ElevatorGoToHeight(elevator, ElevatorConstants.lowHeight)
		);
	}
}
