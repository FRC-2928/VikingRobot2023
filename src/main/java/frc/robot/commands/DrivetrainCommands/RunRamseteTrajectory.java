package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class RunRamseteTrajectory extends RamseteCommand {

  Drivetrain m_drivetrain;
  Trajectory m_trajectory;
  


  /**
   * Creates a new RamseteTrajectoryCommand.
   */
  public RunRamseteTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
    super(
      trajectory,
      drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      DrivetrainConstants.kDriveKinematics,
      drivetrain::setOutputMetersPerSecond,
      drivetrain
    );
    m_drivetrain = drivetrain;
    m_trajectory = trajectory;
  }

  public void initialize() {
    super.initialize();
    m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
    SmartDashboard.putNumber("start traj Y", m_trajectory.getInitialPose().getY());
    SmartDashboard.putNumber("start odom Y", m_drivetrain.getPose().getY());
    m_drivetrain.setPIDSlot(0);
    SmartDashboard.putNumber("start odometry heading", m_drivetrain.getPose().getRotation().getDegrees());
    //SmartDashboard.putNumber("start heading", m_drivetrain.getHeading());
    m_drivetrain.disableMotorSafety();    
  }

  public void execute() {
    super.execute();
    SmartDashboard.putNumber("Odometry X", m_drivetrain.getPose().getX());
    SmartDashboard.putNumber("Odometry Y", m_drivetrain.getPose().getY());
    SmartDashboard.putNumber("Odometry heading", m_drivetrain.getPose().getRotation().getDegrees());
    //SmartDashboard.putNumber("current heading", m_drivetrain.getHeading());
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    //SmartDashboard.putNumber("end heading", m_drivetrain.getHeading());
    m_drivetrain.stopDrivetrain();
    m_drivetrain.enableMotorSafety();
  }

}
