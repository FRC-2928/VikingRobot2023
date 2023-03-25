package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;

public class RunRamseteTrajectory extends RamseteCommand {
	private Drivetrain drivetrain;
	private Trajectory trajectory;

	public RunRamseteTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
		super(
			trajectory,
			drivetrain::getEncoderPose,
			new RamseteController(AutoConstants.ramseteB, AutoConstants.ramseteZeta),
			DrivetrainConstants.driveKinematics,
			drivetrain::setOutputMetersPerSecond,
			drivetrain
		);

		this.drivetrain = drivetrain;
		this.trajectory = trajectory;

		this.addRequirements(drivetrain);
	}

	public void initialize() {
		super.initialize();
		this.drivetrain.resetOdometry(this.trajectory.getInitialPose());
	}

	public void execute() {
		super.execute();
		this.drivetrain.diffDrive.feed();
	}

	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);
		this.drivetrain.halt();
	}
}
