package frc.robot.commands.DrivetrainCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class RunDynamicRamseteTrajectory extends RamseteCommand {
	private Drivetrain drivetrain;
	private Supplier<Trajectory> trajectorySupplier;

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
		this.trajectorySupplier = trajectorySupplier;
	}

	@Override
	public void initialize() {
		super.initialize();
		Pose2d initialPose = this.trajectorySupplier.get().getInitialPose();
		this.drivetrain.resetOdometry(initialPose);
		SmartDashboard.putNumber("Y start traj", initialPose.getY());
	}

	@Override
	public void execute() {
		this.drivetrain.diffDrive.feed();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
