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
		public static final int drivetrainShiftPiston = 1;

		public static final int elevatorLock = 2;
		public static final int armLock = 0;
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
		// TODO: make more accurate
		public static final Gains armGains = new Gains(0.01, 0.02, 0.0015, 0.0, 100, 0.50);
		public static final double lowPositionCone = -67.5;
		public static final double lowPositionCube = -73.5;
		public static final double midPosition = -22.6;
		public static final double highPosition = 0 - 4;
		public static final double inPosition = -109;
		public static final double defaultPower = .4;
		
		//TODO: make encoder reset right
		public static final double armLimitSwitchEncoderValue = -114;
		public static final double doubleSubstationCone = -4 - 4;
		public static final double doubleSubstationCube = -7 - 3;
		
		public static final double homeAngleLimit = -120;
		public static final double maxAngleLimit = 18.0;
	}

	public static final class DrivetrainConstants {
		public static final double trackWidthMeters = 0.7; // Placeholder
		public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackWidthMeters);

		// TODO: find these
		public static final double toHighDistance = .2;
		public static final double honeToMidDistance = .6;

		public static final int encoderCPR = 2048;
		public static final double wheelDiameterMeters = 0.1016;

		// Assumes the encoders are directly mounted on the wheel shafts
		public static final double kEncoderDistancePerPulse = (wheelDiameterMeters * Math.PI) / (double)encoderCPR;

		public static final double unitsPerRevolution = 2048;

		public static final double highGearRatio = 5.4;
		public static final double lowGearRatio = 8.82;

		public static final TrapezoidProfile.Constraints kTrapezoidProfileConstraints = new TrapezoidProfile.Constraints(
			AutoConstants.maxSpeedMetersPerSecond,
			AutoConstants.maxAccelMetersPerSecondSquared
		);

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
		// TODO: tune when weight is added
		public static final Gains GainsBalance = new Gains(0.08, 0.005, 0.0025, 0, 0, 0.3);
		public static final Gains GainsAlignBalance = new Gains(1.1, 0.0, 0.01, 0.0, 0, 0.3);
		public static final Gains GainsTurnto = new Gains(.08,0.001,0.01,0.0,0,0.3);

		//trajectories and localization

		// 0,0 is blue tag 8

		public static final double fieldWidthYMeters = 8.102;
		public static final double fieldLengthXMeters = 13.436;

		public static final double yOffsetField = 4.051;
		//offset for limelight and length of robot
		public static final double xOffsetField = 6.718 + .91;

		public static final double manualDriveMultiplier = 1;
		public static final double manualTurnMultiplier = .6;

		public static final double reductFactor = 0.5;
		public static final double reductFactorRotation = 0.75;
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
		public static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(
			ksVolts,
			kvVoltSecondsPerMeter,
			kaVoltSecondsSquaredPerMeter
		);
		public static final SimpleMotorFeedforward feedForwardL = new SimpleMotorFeedforward(
			ksVolts,
			kvVoltSecondsPerMeter,
			kaVoltSecondsSquaredPerMeter
		);
		public static final SimpleMotorFeedforward feedForwardR = new SimpleMotorFeedforward(
			ksVolts,
			kvVoltSecondsPerMeter,
			kaVoltSecondsSquaredPerMeter
		);
		public static final double maxVolts = 10;
		public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
			feedForward,
			DrivetrainConstants.driveKinematics,
			maxVolts
		);

		public static final double maxSpeedMetersPerSecond = 1.0;
		public static final double maxAccelMetersPerSecondSquared = 1.0;

		// Setup trajectory constraints
		public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
			maxSpeedMetersPerSecond,
			maxAccelMetersPerSecondSquared
		)
			.setKinematics(DrivetrainConstants.driveKinematics)
			.addConstraint(autoVoltageConstraint);

		public static final TrajectoryConfig trajectoryConfigReversed = new TrajectoryConfig(
			maxSpeedMetersPerSecond,
			maxAccelMetersPerSecondSquared
		)
			.setKinematics(DrivetrainConstants.driveKinematics)
			.addConstraint(autoVoltageConstraint)
			.setReversed(true);

		// Reasonable baseline values for a RAMSETE follower in units of meters and
		// seconds
		public static final double ramseteB = 2;
		public static final double ramseteZeta = 0.7;
	}
}
