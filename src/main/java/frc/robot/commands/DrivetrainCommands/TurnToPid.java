// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToPid extends PIDCommand {
  /** Creates a new TurnTo. */
  public TurnToPid(double angle, double speed, Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(DrivetrainConstants.GainsTurnto.P,DrivetrainConstants.GainsTurnto.I,DrivetrainConstants.GainsTurnto.D),
        // This should return the measurement
        () -> drivetrain.readYaw(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          if(drivetrain.readYaw() > angle){drivetrain.tankDriveVolts(-output*speed,output*speed);}
          else{drivetrain.tankDriveVolts(output*speed,-output*speed);}
        });
        this.m_controller.setTolerance(0.3);
        this.m_controller.setSetpoint(0.0);
        this.m_controller.calculate(0.0);
        this.addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.m_controller.atSetpoint();
  }
}
