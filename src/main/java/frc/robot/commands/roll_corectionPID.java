// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
//import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
//import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Roll_corectionPID extends PIDCommand {
  /** Creates a new roll_corection. */
  public Drivetrain drive;
  public Roll_corectionPID(Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(DrivetrainConstants.GainsRollBalance.P, DrivetrainConstants.GainsRollBalance.I,DrivetrainConstants.GainsRollBalance.D),
        // This should return the measurement
        
        () -> drivetrain.m_pigeon.getRoll(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          SmartDashboard.putNumber("Roll",drivetrain.m_pigeon.getRoll());
          SmartDashboard.putNumber("Output",output);
          drivetrain.tankDriveVolts(-output,output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(2, 10);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
