// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ArmCommands.ArmGoToPosition;
import frc.robot.commands.ElevatorCommands.ElevatorGoToHeight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveElevatorAndArm extends SequentialCommandGroup {
  /** Creates a new GoHigh. */
  public MoveElevatorAndArm(Elevator elevator, Arm arm, double elevatorGoal, double armGoal) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //if going down
    if (elevator.getEncoderTicks() > elevatorGoal){
      addCommands(new ArmGoToPosition(arm, armGoal),
                  new ElevatorGoToHeight(elevator, elevatorGoal));
    } else {
      addCommands(new ElevatorGoToHeight(elevator, elevatorGoal),
                  new ArmGoToPosition(arm, armGoal));
    }
    
  }
}
