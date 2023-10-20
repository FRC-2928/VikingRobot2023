package frc.robot;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;

import javax.imageio.ImageIO;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.*;

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
	public static enum GamePiece {
		Cone, // not purple
		Cube; // purple

        public String getImageName() {
            switch(this) {
                case Cone: return "cone";
                case Cube: return "cube";
                default: return null; // unreachable
            }
        }
	}

	public static class Gains {
		// Proportional gain for closed loop. This is multiplied by closed loop error in sensor units.
		// Note the closed loop output interprets a final value of 1023 as full output.
		// So use a gain of ‘0.25’ to get full output if err is 4096u (Mag Encoder 1 rotation)
		public final double P;

		// Integral gain for closed loop.
		// This is multiplied by closed loop error in sensor units every PID Loop.
		public final double I;

		// Derivative gain for closed loop.
		// This is multiplied by derivative error (sensor units per PID loop).
		public final double D;

		// Feed Forward gain for Closed loop.
		// If using velocity, motion magic, or motion profile, use (1023 * duty-cycle / sensor-velocity-sensor-units-per-100ms)
		public final double F;

		// Integral Zone can be used to auto clear the integral accumulator if the sensor position
		// is too far from the target. This prevent unstable oscillation if the kI is too large.
		// Value is in sensor units.
		public final int iZone;

		// Absolute max motor output during closed-loop control modes only.
		// A value of ‘1’ represents full output in both directions.
		public final double peakOutput;

		public Gains(final double _kP, final double _kI, final double _kD, final double _kF, final int _kIzone, final double _kPeakOutput) {
			this.P = _kP;
			this.I = _kI;
			this.D = _kD;
			this.F = _kF;
			this.iZone = _kIzone;
			this.peakOutput = _kPeakOutput;
		}
	}

	public static final class PneumaticIDs {
		public static final int drivetrainShiftPiston = 1;

		public static final int elevatorLock = 2;
		public static final int armLock = 0;
	}

	public static final class CANBusIDs {
		// Drivetrain, right side
		public static final int DrivetrainRightBackTalon = 19;
		public static final int DrivetrainRightFrontTalon = 18;

		// Drivetrain, left side
		public static final int DrivetrainLeftFrontTalon = 13;
		public static final int DrivetrainLeftBackTalon = 10;

		// Elevator
		public static final int ElevatorTalon = 6;

		// Arm
		public static final int ArmTalonLeader = 7;
		public static final int ArmTalonFollower = 8;

		// Intake
		public static final int IntakeTalon = 9;

		// Sensors
		public static final int Pigeon = 0;
		public static final int ArmEncoder = 5;
	}

	public static final class LimelightFXConstants {
        public static final boolean disable = false;

		public static final LimelightFX.Color coneColor = new LimelightFX.Color(0, 0, 0);

		public static final LimelightFX.Color cubeFillColor = new LimelightFX.Color(102, 0, 255);
		public static final LimelightFX.Color cubeBorderColor = new LimelightFX.Color(174, 107, 255);

        public static final boolean useImageSignals = true;

        public static BufferedImage image(final String name) {
            if(LimelightFXConstants.imageCache.containsKey(name)) return LimelightFXConstants.imageCache.get(name);

            try {
				return ImageIO.read(new File(
                    Filesystem.getDeployDirectory()
                        .toPath()
                        .resolve("img")
                        .resolve(name + ".bmp")
                        .toAbsolutePath()
                        .toString()
                ));
			} catch(final IOException e) {
				Log.error(e);
				return null;
			}
        }

        private static HashMap<String, BufferedImage> imageCache = new HashMap<>();
	}

	public static final class GlassMechanismConstants {
		// Length & thickness measurements in meters

		public static final class Dimension {
			public final Color8Bit color;
			public final double thickness;
			public final double length;
			public final double angle;

			Dimension(final Color8Bit color, final double thickness, final double length, final double angle) {
				this.color = color;
				this.thickness = thickness;
				this.length = length;
				this.angle = angle;
			}
		}

		public static final Dimension elevator = new Dimension(
			new Color8Bit(128, 128, 128),
			0.05,
			1,
			30
		);
		public static final Dimension elevatorExtension = new Dimension(
			new Color8Bit(192, 192, 192),
			0.025,
			0.5,
			30
		);
		public static final Dimension armColor = new Dimension(
			new Color8Bit(64, 255, 255),
			0.05,
			1,
			0
		);
		public static final Dimension intakeColor = new Dimension(
			new Color8Bit(255, 255, 255),
			0.05,
			0.25,
			90
		);
	}

	public static final class IntakeConstants {
		public static final double intakePower = -0.85;
		public static final double intakeCubePower = -0.3;
		public static final double shootConePower = 0.85;
		public static final double shootCubePower = 0.5;
	}

	public static final class ElevatorConstants {
		public static final Gains elevatorGains = new Gains(0.015, 0.0, 0.0, 0.0, 100, 0.50);

		public static final int averageLockIntervalTicks = -1925; // distance in encoder ticks between locking piston clicks
		// public static final int exitHeight = -10000; // min height to allow arm movement
		// public static final int driveHeight = -9780;
		public static final int substationHeight = 8545 - 2 * ElevatorConstants.averageLockIntervalTicks;

		public static final double lowHeight = 13674;
		public static final double highHeight = -11500 - 500;
		public static final double stashHeight = 0;
		public static final double defaultPower = 0.2;
		public static final double homingPower = -0.55;

		public static final int homeOffset = 1900;
		public static final int topOffset = -12176;
		public static final int bottomSoftLimit = 11000;
		public static final int topSoftLimit = -13600;

		// public static final int elevatorForStart = -1300;
	}

	public static final class ArmConstants {
		public static final Gains armGains = new Gains(0.01, 0.02, 0.0015, 0.0, 100, 0.50);

		public static final double lowPositionCone = -67.5;
		public static final double lowPositionCube = -75;
		public static final double midPosition = -22.2 - 4;
		public static final double highPosition = 0 - 4;
		public static final double highCubeAuto = 0 + 6;
		public static final double inPosition = -109;
		public static final double defaultPower = .4;

		public static final double armLimitSwitchOffset = -114 - 3.428;
		public static final double doubleSubstationCone = -4 - 4;
		public static final double doubleSubstationCube = -7 - 3;

		public static final double homeAngleLimit = -120;
		public static final double maxAngleLimit = 18.0;
	}

	public static final class DrivetrainConstants {
		public static final double trackWidthMeters = 0.7; // Placeholder
		public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(DrivetrainConstants.trackWidthMeters);

		public static final double toHighDistance = .2;

		public static final int encoderCPR = 2048;
		public static final double wheelDiameterMeters = 0.1016;

		// Assumes the encoders are directly mounted on the wheel shafts
		public static final double kEncoderDistancePerPulse = (DrivetrainConstants.wheelDiameterMeters * Math.PI) / (double)DrivetrainConstants.encoderCPR;

		public static final double unitsPerRevolution = 2048;

		public static final double highGearRatio = 5.4;
		public static final double lowGearRatio = 8.82;

		public static final double poleVertical = 20;
		public static final double poleHorizontal = 0;

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
		public static final Gains GainsBalance = new Gains(0.085, 0.005, 0.0025, 0, 0, 0.3);
		public static final Gains GainsAlignBalance = new Gains(1.1, 0.0, 0.01, 0.0, 0, 0.3);
		public static final Gains GainsTurnto = new Gains(.08,0.001,0.01,0.0,0,0.3);
//couldChangePToBeLowerOrDToBeHigherButOnlyATeensyTinsyBitOfDBecauseDScaresMe
		public static final Gains GainsTurnRetroflective = new Gains(.0095, 0, 0.001, 0, 0, .5);
		public static final Gains GainsGoRetroflective = new Gains(.1, 0, 0, 0, 0, .5);

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
			AutoConstants.ksVolts,
			AutoConstants.kvVoltSecondsPerMeter,
			AutoConstants.kaVoltSecondsSquaredPerMeter
		);
		public static final SimpleMotorFeedforward feedForwardL = new SimpleMotorFeedforward(
			AutoConstants.ksVolts,
			AutoConstants.kvVoltSecondsPerMeter,
			AutoConstants.kaVoltSecondsSquaredPerMeter
		);
		public static final SimpleMotorFeedforward feedForwardR = new SimpleMotorFeedforward(
			AutoConstants.ksVolts,
			AutoConstants.kvVoltSecondsPerMeter,
			AutoConstants.kaVoltSecondsSquaredPerMeter
		);
		public static final double maxVolts = 10;
		public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
			AutoConstants.feedForward,
			DrivetrainConstants.driveKinematics,
			AutoConstants.maxVolts
		);

		public static final double maxSpeedMetersPerSecond = 1.0;
		public static final double maxAccelMetersPerSecondSquared = 1.0;

		// Setup trajectory constraints
		public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
			AutoConstants.maxSpeedMetersPerSecond,
			AutoConstants.maxAccelMetersPerSecondSquared
		)
			.setKinematics(DrivetrainConstants.driveKinematics)
			.addConstraint(AutoConstants.autoVoltageConstraint);

		public static final TrajectoryConfig trajectoryConfigReversed = new TrajectoryConfig(
			AutoConstants.maxSpeedMetersPerSecond,
			AutoConstants.maxAccelMetersPerSecondSquared
		)
			.setKinematics(DrivetrainConstants.driveKinematics)
			.addConstraint(AutoConstants.autoVoltageConstraint)
			.setReversed(true);

		// Reasonable baseline values for a RAMSETE follower in units of meters and
		// seconds
		public static final double ramseteB = 2;
		public static final double ramseteZeta = 0.7;
	}
}
