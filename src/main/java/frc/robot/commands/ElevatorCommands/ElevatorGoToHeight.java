// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorGoToHeight extends PIDCommand {

  Elevator m_elevator;
  double m_goalHeight;

  /** Creates a new ElevatorGoToHeight. */
  public ElevatorGoToHeight(Elevator elevator, double goalHeight) {
    super(
        // The controller that the command will use
        new PIDController(ElevatorConstants.elevatorGains.P, ElevatorConstants.elevatorGains.I, ElevatorConstants.elevatorGains.D),
        // This should return the measurement
        () -> elevator.getEncoderTicks(),
        // This should return the setpoint (can also be a constant)
        () -> goalHeight,
        // This uses the output
        output -> {
          // Use the output here
          elevator.setPower(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    addRequirements(m_elevator);
    m_goalHeight = goalHeight;
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(30);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
