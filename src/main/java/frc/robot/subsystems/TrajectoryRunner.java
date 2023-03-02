package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;

public class TrajectoryRunner {
	public static enum Direction {
		Left,
		Right,
		Center
	}

    /**
	 * Generates a dynamic trajectory starting at the current pose of the
	 * robot, as determined by the Limelight looking at the AprilTags.
	 * The trajectory will end at the current in-view apriltag, or to the
	 * left or right of it as directed.
	 *
	 * @param direction whether the robot should end at the center, left,
	 *                  or right of the apriltag.
	 *
	 * @return The generated Trajectory object
	 */
	public static Trajectory generateLocalTrajectory(Drivetrain drivetrain, Direction direction) {
		Trajectory trajectory = new Trajectory();

		// Get the aprilTag that the robot is looking at
		int aprilTagID = drivetrain.getAprilTagID();

		// Default trajectory if no limelight target is to move back 0.5 meters
		Pose2d startPose = drivetrain.getEncoderPose();
		Pose2d endPose = startPose.plus(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d()));
		
		TrajectoryConfig config = AutoConstants.kTrajectoryConfigReversed;

		List<Translation2d> waypoints = new ArrayList<>();

		if(!drivetrain.hasValidLimelightTarget()) {
			Log.warning("LocalTrajectory failed: No limelight target");
			
			Robot.instance.robotContainer.driverOI.signalError();

			return null;
		} else if(!FieldConstants.aprilTags.containsKey(aprilTagID)) {
			throw new Error("Attempted to go to an AprilTag that does not exist! Id #" + aprilTagID);
		} else {
			// Get the aprilTag that the robot is looking at and it's pose relative to the tag.
			startPose = drivetrain.getLimelightPoseRelative();
			// Move forward config
			config = AutoConstants.kTrajectoryConfig;

			// Now get the pose
			Pose2d tag = FieldConstants.aprilTags.get(aprilTagID).toPose2d();

			if(direction == Direction.Left) endPose = tag.plus(FieldConstants.leftOffset);
			else if(direction == Direction.Right) endPose = tag.plus(FieldConstants.rightOffset);
			else if(direction == Direction.Center) endPose = tag.plus(FieldConstants.centerOffset);
		}

		trajectory = TrajectoryGenerator.generateTrajectory(
			startPose,
			waypoints,
        	endPose,
			config
		);

        return trajectory;
	}

    /**
	 * Generate a trajectory following Ramsete command
	 *
	 * This is very similar to the WPILib RamseteCommand example. It uses
	 * constants defined in the Constants.java file. These constants were
	 * found empirically by using the frc-characterization tool.
	 *
	 * @return A SequentialCommand that sets up and executes a trajectory following Ramsete command
	 */
  	public static Command generateRamseteCommand(Drivetrain drivetrain, Supplier<Trajectory> trajectory) {
		RamseteCommand ramseteCommand = new RamseteCommand(
			new Trajectory(),
			drivetrain::getPose,
			new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
			DrivetrainConstants.kDriveKinematics,
			drivetrain::setOutputMetersPerSecond,
			drivetrain
		);

		// TODO: Test
		Optional<Trajectory> traj = Optional.empty();

		// Set up a sequence of commands
		// First, we want to reset the drivetrain odometry
		return new InstantCommand(() -> {
			traj.inner = trajectory.get();
			try {
				Field field = ramseteCommand.getClass().getDeclaredField("m_trajectory");
				field.setAccessible(true);
				field.set(ramseteCommand, traj);
			} catch(Exception e) {
				Log.error(e);
			}
			drivetrain.resetOdometry(traj.inner.getInitialPose());
		}, drivetrain)
			// next, we run the actual ramsete command
			.andThen(new ConditionalCommand(ramseteCommand, new InstantCommand(), () -> traj.inner != null))
			// make sure that the robot stops
			.andThen(new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0), drivetrain));
	}

    public static Trajectory loadTrajectory(String trajectoryJSON) {
		Path trajectoryPath = Filesystem
			.getDeployDirectory()
			.toPath()
			.resolve("paths/output/" + trajectoryJSON + ".wpilib.json");

		try {
			return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			Log.error(ex);
			return null;
		}
	}
}
