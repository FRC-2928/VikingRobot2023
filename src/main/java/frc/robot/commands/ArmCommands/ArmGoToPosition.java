// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmGoToPosition extends PIDCommand {
  /** Creates a new ArmGoToPositin. */
  public ArmGoToPosition(Arm arm, double goalPosition) {
    super(
        // The controller that the command will use
        new PIDController(ArmConstants.armGains.P, ArmConstants.armGains.I, ArmConstants.armGains.D),
        // This should return the measurement
        () -> arm.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> goalPosition,
        // This uses the output
        output -> {
          // Use the output here
          arm.setPower(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(30);
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
