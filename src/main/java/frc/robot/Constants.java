package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class PneumaticIDs {
		// Solenoids
		// public static final int kDrivetrainShiftSolenoid = 0;
		public static final int kRampSolenoid = 1;
		public static final int kClimberSolenoid = 2;

		public static final int kDrivetrainShiftSolenoidLow = 2;
		public static final int kDrivetrainShiftSolenoidHigh = 3;
		// public static final int kRampSolenoidOpen = 2;
		// public static final int kRampSolenoidClosed = 3;

	}

	public static final class CANBusIDs {
		// Drivetrain, right side
		public static final int DrivetrainRightBackTalonFX = 0;
		public static final int DrivetrainRightFrontTalonFX = 1;
		// Drivetrain, left side
		public static final int DrivetrainLeftFrontTalonFX = 14;
		public static final int DrivetrainLeftBackTalonFX = 15;

		// Sensors
		public static final int kPigeonIMU = 3;
	}

	public static final class AutoConstants {
		// Setup trajectory constraints
		public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
				DrivetrainConstants.kMaxSpeedMetersPerSecond,
				DrivetrainConstants.kMaxAccelMetersPerSecondSquared)
				.setKinematics(DrivetrainConstants.kDriveKinematics)
				.addConstraint(DrivetrainConstants.kAutoVoltageConstraint);

		// Reasonable baseline values for a RAMSETE follower in units of meters and
		// seconds
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;
	}

	public static final class DrivetrainConstants {

		

		// kS (static friction), kV (velocity), and kA (acceleration)
		public static final double ksVolts = 0.6024;
		public static final double kvVoltSecondsPerMeter = 0.21907;
		public static final double kaVoltSecondsSquaredPerMeter = 0.0096252;

		// Feedforward contraints
		public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(ksVolts,
				kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

		public static final double kTrackWidthMeters = 0.7; // Placeholder
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
				kTrackWidthMeters);

		public static final double k_MaxVolts = 10;
		public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				kFeedForward,
				kDriveKinematics,
				k_MaxVolts);

		public static final boolean kGyroReversed = true;

		public static final int encoderCPR = 2048;
		public static final double kWheelDiameterMeters = 0.1015;

		public static final double kEncoderDistancePerPulse =
				// Assumes the encoders are directly mounted on the wheel shafts
				(kWheelDiameterMeters * Math.PI) / (double) encoderCPR;

		public static final double kUnitsPerRevolution = 2048;

		// TO DO change to correct values
		public static final double highGearRatio = 5;
		public static final double lowGearRatio = 10.71;

		public static final double kMaxSpeedMetersPerSecond = 2.0;
		public static final double kMaxAccelMetersPerSecondSquared = 2.0;

		public static final TrapezoidProfile.Constraints kTrapezoidProfileConstraints = new TrapezoidProfile.Constraints(
				kMaxSpeedMetersPerSecond, kMaxAccelMetersPerSecondSquared);

		// PID Constants
		// The WPILib feedforward has kS (static friction), kV (velocity), and kA
		// (acceleration) terms
		// whereas the Talon SRX / Spark MAX kF is only a kV (velocity) feedforward.
		// kp, ki, kd, kf, iz, peak output
		public static final Gains GainsProfiled = new Gains(0.16, 0, 0, 0, 0, 1.00);

		/**
		 * PID Gains may have to be adjusted based on the responsiveness of control
		 * loop.
		 * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity
		 * units at 100% output
		 * Not all set of Gains are used in this project and may be removed as desired.
		 * 
		 * kP kI kD kF Iz PeakOut
		 */
		public static final Gains GainsDistance = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
		public static final Gains GainsTurning = new Gains(0.10, 0.0, 0.0, 0.0, 200, 1.00);
		public static final Gains GainsVelocity = new Gains(0.1, 0.001, 5, 1023.0 / 20660.0, 300, 1.00);
		public static final Gains GainsMotProf = new Gains(1.0, 0.0, 0.0, 1023.0 / 20660.0, 400, 1.00);
		public static final Gains GainsBalance = new Gains(0.1, 0.0, 0.0, 0, 0, 0.3);
		public static final Gains GainsRollBalance = new Gains(1, 0, 0.03, 0.0, 0, 0.3);

		// public static final double kDistanceToleranceMeters = 0.1;
		// public static final double kVelocityToleranceMeters = 0.1;


		//trajectories and localization

		// 0,0 is blue tag 8

		public static final double fieldWidthYMeters = 8.102;
		public static final double fieldLengthXMeters = 13.436;

		public static final double yOffsetField = 4.051;
		//offset for limelight and length of robot
		public static final double xOffsetField = 6.718 + .91;
		public static final double arcadeDriveMultiplier = 0.8;

		public static final TrajectoryConfig kTrajectoryConfig = 
			new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelMetersPerSecondSquared)
			.setKinematics(kDriveKinematics)
			.addConstraint(kAutoVoltageConstraint);

	}
}
