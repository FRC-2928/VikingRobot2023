// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Log;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToPid extends ProfiledPIDCommand {

  Drivetrain drivetrain;

  /** Creates a new TurnTo. */
  public TurnToPid(double angle, Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new ProfiledPIDController(DrivetrainConstants.GainsTurnto.P,DrivetrainConstants.GainsTurnto.I,DrivetrainConstants.GainsTurnto.D,
                new TrapezoidProfile.Constraints(360, 200)),
        // This should return the measurement
        () -> drivetrain.readYaw(),
        // This should return the setpoint (can also be a constant)
        () -> new TrapezoidProfile.State(angle, 0),
        // This uses the output
        (output, setpoint) -> {
          //if(drivetrain.readYaw() > angle){drivetrain.tankDriveVolts(-output,output);}
          // else{drivetrain.tankDriveVolts(output,-output);}
          drivetrain.tankDriveVolts(output, -output);
        });
        this.m_controller.setTolerance(3, 10);
        this.addRequirements(drivetrain);
        this.drivetrain = drivetrain;
  }

  public void initialize() {
		super.initialize();
    Log.writeln("TurnToPid Start Pose: " + this.drivetrain.getEncoderPose());
    this.drivetrain.disableMotorSafety();
  }

  @Override
	public void end(boolean interrupted) {
		super.end(interrupted);
		Log.writeln("TurnToPid End Pose: " + this.drivetrain.getEncoderPose());
		this.drivetrain.enableMotorSafety();
	}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.m_controller.atGoal();
  }
}
