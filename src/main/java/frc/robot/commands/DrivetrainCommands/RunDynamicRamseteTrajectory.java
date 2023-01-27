// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DrivetrainCommands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Log;

public class RunDynamicRamseteTrajectory extends RamseteCommand {
	private Drivetrain drivetrain;
  private Trajectory trajectory;

  /** Creates a new RunDynamicRamseteTrajectory. */
  public RunDynamicRamseteTrajectory(Drivetrain drivetrain, Supplier<Trajectory> trajectorySupplier) {
    super(
			trajectorySupplier.get(),
			drivetrain::getEncoderPose,
			new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
			DrivetrainConstants.kDriveKinematics,
			drivetrain::setOutputMetersPerSecond,
			drivetrain
		);
		this.drivetrain = drivetrain;
    this.trajectory = trajectorySupplier.get();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
      Pose2d initialPose = this.trajectory.getInitialPose();
      this.drivetrain.resetOdometry(initialPose);
      this.drivetrain.disableMotorSafety();   
      SmartDashboard.putNumber("Y start traj", initialPose.getY());
      // printTrajectory();   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivetrain.enableMotorSafety();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void printTrajectory() {
      Log.writeln("Initial Pose: " + this.trajectory.getInitialPose());
      
      List<State> states = this.trajectory.getStates();
      for (int i = 1; i < states.size(); i++) {
        var state = states.get(i);
        Log.writeln("Time:" + state.timeSeconds + " Velocity:" + state.velocityMetersPerSecond);
      }  
  }
}
