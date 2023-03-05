package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
	public enum GamePiece {
		Cone, // not purple
		Cube, // purple
	}
	public static final class PneumaticIDs {
		public static final int kDrivetrainShiftPiston = 1;

		public static final int kElevatorLock = 2;
		public static final int kArmLock = 0;
	}

	public static final class CANBusIDs {
		// Drivetrain, right side
		public static final int DrivetrainRightBackTalonFX = 19;
		public static final int DrivetrainRightFrontTalonFX = 18;

		//public static final int DrivetrainLeftBackTalonFX = 0;
		//public static final int DrivetrainLeftFrontTalonFX = 1;

		// Drivetrain, left side
		public static final int DrivetrainLeftFrontTalonFX = 13;
		public static final int DrivetrainLeftBackTalonFX = 10;

		//public static final int DrivetrainRightFrontTalonFX = 14;
		//public static final int DrivetrainRightBackTalonFX = 15;

		//Arm
		public static final int ArmTalon1 = 7;
		public static final int ArmTalon2 = 8;

		//Intake
		public static final int IntakeTalon1 = 9;

		public static final int ElevatorTalon1 = 6;

		// Sensors
		public static final int PigeonIMU = 0;

		public static final int ArmEncoder = 5;
	}

	public static final class LimelightConstants {
		public static final double tagGoalY = 8.25;
		public static final double tagGoalX = 0;
	}

	public static final class IntakeConstants {
		public static final double intakePower = -0.85;
		public static final double intakeCubePower = -0.3;
		public static final double shootConePower = 0.85;
		public static final double shootCubePower = 0.5;
	}

	public static final class ElevatorConstants {
		public static final Gains elevatorGains = new Gains(0.01, 0.0, 0.0, 0.0, 100, 0.50);
		//public static final double lowHeight = 9000;
		public static final double lowHeight = 13674;
		public static final double highHeight = -11500 - 2000;
		public static final double stashHeight = 0;
		public static final double defaultPower = 0.2;
		public static final double homingPower = -0.4;

		public static final int homeOffset = 1900;
		public static final int topOffset = -11650;
		public static final int bottomSoftLimit = 11000;
		public static final int topSoftLimit = -11600;

		public static final int elevatorForStart = -1300;


		public static final int exitHeight = -10000; // min height to allow arm movement
		public static final int driveHeight = -9780;

		public static final int averageLockIntervalTicks = -1925; // distance in encoder ticks between locking piston clicks
		public static final int elevatorForHumanPlayer = 8545 - 2 * averageLockIntervalTicks;
	}

	public static final class ArmConstants {
		public static final Gains armGains = new Gains(0.01, 0.02, 0.0, 0.0, 100, 0.50);
		public static final double lowPositionCone = -59.4;
		public static final double lowPositionCube = -73.5;
		public static final double midPosition = -22.6;
		public static final double highPosition = 0;
		public static final double inPosition = -110;
		public static final double defaultPower = .4;
		public static final double humanPlayerPosition = -9.5;

		//public static final double homeAngle = -116;
		
		public static final double homeAngleLimit = -119;
		public static final double maxAngleLimit = 18.0;
	}

	public static final class DrivetrainConstants {
		public static final double kTrackWidthMeters = 0.7; // Placeholder
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
				kTrackWidthMeters);

		public static final boolean kGyroReversed = true;

		// TODO: find these
		public static final double honeToHighDistance = .2;
		public static final double honeToMidDistance = .6;

		public static final int encoderCPR = 2048;
		public static final double kWheelDiameterMeters = 0.1016;

		public static final double kEncoderDistancePerPulse =
				// Assumes the encoders are directly mounted on the wheel shafts
				(kWheelDiameterMeters * Math.PI) / (double) encoderCPR;

		public static final double kUnitsPerRevolution = 2048;

		// for sweetpants
		public static final double highGearRatio = 5.4;
		public static final double lowGearRatio = 8.82;

		// for competition robot
		// public static final double highGearRatio = 5.68;
		// public static final double lowGearRatio = 8.82;

		public static final TrapezoidProfile.Constraints kTrapezoidProfileConstraints = new TrapezoidProfile.Constraints(
				AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelMetersPerSecondSquared);

		// PID Constants
		// The WPILib feedforward has kS (static friction), kV (velocity), and kA
		// (acceleration) terms
		// whereas the Talon SRX / Spark MAX kF is only a kV (velocity) feedforward.
		// kp, ki, kd, kf, iz, peak output

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
		public static final Gains GainsBalance = new Gains(0.08, 0.005, 0.0025, 0, 0, 0.3);
		public static final Gains GainsAlignBalance = new Gains(1.1, 0.0, 0.01, 0.0, 0, 0.3);
		public static final Gains GainsTurnto = new Gains(.08,0.001,0.01,0.0,0,0.3);
		public static final Gains GainsApproachTurn = new Gains(.1,0,0.0,0.0,0,0.3);
		public static final Gains GainsApproach = new Gains(.65,0.016,0.0,0.0,0,0.3);
		// public static final double kDistanceToleranceMeters = 0.1;
		// public static final double kVelocityToleranceMeters = 0.1;

		//trajectories and localization

		// 0,0 is blue tag 8

		public static final double fieldWidthYMeters = 8.102;
		public static final double fieldLengthXMeters = 13.436;

		public static final double yOffsetField = 4.051;
		//offset for limelight and length of robot
		public static final double xOffsetField = 6.718 + .91;
		public static final double manualDriveMultiplier = 1;
		public static final double manualTurnMultiplier = .6;
	}

	public static final class AutoConstants {
		// public static final Gains GainsAuto = new Gains(0.08, 0.001, 0, 0, 0, 1.00);
		public static final Gains GainsAuto = new Gains(0.06, 0.001, 0.04, 0, 0, 1.00);

		// kS (static friction), kV (velocity), and kA (acceleration)
		// public static final double ksVolts = 0.3024;
		// public static final double kvVoltSecondsPerMeter = 0.21907;
		// public static final double kaVoltSecondsSquaredPerMeter = 0.0096252;

		public static final double ksVolts = 0.073457;
		public static final double kvVoltSecondsPerMeter = 3.7486;
		public static final double kaVoltSecondsSquaredPerMeter = 0.023529;

		// Feedforward contraints
		public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(ksVolts,
				kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
		public static final SimpleMotorFeedforward kFeedForwardL = new SimpleMotorFeedforward(ksVolts,
				kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
		public static final SimpleMotorFeedforward kFeedForwardR = new SimpleMotorFeedforward(ksVolts,
				kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
		public static final double k_MaxVolts = 10;
		public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				kFeedForward,
				DrivetrainConstants.kDriveKinematics,
				k_MaxVolts);

		public static final double kMaxSpeedMetersPerSecond = 1.0;
		public static final double kMaxAccelMetersPerSecondSquared = 1.0;

		// Setup trajectory constraints
		public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
				kMaxSpeedMetersPerSecond,
				kMaxAccelMetersPerSecondSquared)
				.setKinematics(DrivetrainConstants.kDriveKinematics)
				.addConstraint(kAutoVoltageConstraint);

		public static final TrajectoryConfig kTrajectoryConfigReversed = new TrajectoryConfig(
			kMaxSpeedMetersPerSecond,
			kMaxAccelMetersPerSecondSquared)
			.setKinematics(DrivetrainConstants.kDriveKinematics)
			.addConstraint(kAutoVoltageConstraint)
			.setReversed(true);

		// Reasonable baseline values for a RAMSETE follower in units of meters and
		// seconds
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;
	}
}
