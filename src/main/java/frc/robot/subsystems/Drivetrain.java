package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.sim.DrivebaseSimFX;
import frc.robot.subsystems.Transmission.GearState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class Drivetrain extends SubsystemBase {
	public final WPI_TalonFX rightLeader = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainRightBackTalonFX);
	public final WPI_TalonFX leftLeader = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainLeftBackTalonFX);
	public final WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainRightFrontTalonFX);
	public final WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainLeftFrontTalonFX);

  	private final Limelight limelight = new Limelight();

	public final DifferentialDrive diffDrive;

	// TODO: make this work
	public boolean brakeOverride = false;

	private Arm m_arm;


	private WPI_Pigeon2 pigeon = new WPI_Pigeon2(Constants.CANBusIDs.PigeonIMU);

	// Drivetrain kinematics, feed it width between wheels
	private SimpleMotorFeedforward feedForwardL;
	private SimpleMotorFeedforward feedForwardR;

	private double offset;
	private double speedMultiplier = 1;
	private boolean speedMultiplierOn = false;

	private MedianFilter filterVertical = new MedianFilter(10);

	private DifferentialDriveOdometry odometry;
	private DifferentialDrivePoseEstimator poseEstimator;

	private final Field2d field2d = new Field2d();
	private final Field2d fieldEstimated = new Field2d();
	private final Field2d fieldLimelight = new Field2d();

	private DrivebaseSimFX driveSim = new DrivebaseSimFX(rightLeader, leftLeader, pigeon);

	// -----------------------------------------------------------
	// Initialization
	// -----------------------------------------------------------
	public Drivetrain(Arm arm) {
		// Configure Talon motors
		this.configureMotors();

		this.setWheelPIDF();

		this.diffDrive = new DifferentialDrive(rightLeader, leftLeader);

		this.feedForwardL = AutoConstants.feedForwardL;
		this.feedForwardR = AutoConstants.feedForwardR;

		this.resetEncoders();
		this.zeroGyro();

		m_arm = arm;

		if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
			this.pigeon.setYaw(0);
		} else {
			this.pigeon.setYaw(180);
		}

		// Start with default Pose2d(0, 0, 0)
		this.odometry = new DifferentialDriveOdometry(new Rotation2d(this.readYaw()), 0, 0);
		this.poseEstimator = new DifferentialDrivePoseEstimator(DrivetrainConstants.driveKinematics,
									new Rotation2d(this.readYaw()), 0, 0, this.getLimelightPose2d());

		this.field2d.setRobotPose(this.getEncoderPose());
		SmartDashboard.putData("Encoder Pose", this.field2d);

		this.fieldEstimated.setRobotPose(this.getEstimatedPose());
		SmartDashboard.putData("Estimated Pose", this.fieldEstimated);

		this.fieldLimelight.setRobotPose(this.getLimelightPose2d());
		SmartDashboard.putData("Limelight Pose", this.fieldLimelight);
	}

	public void configureMotors() {
		// Configure the motors
		for(TalonFX fx : new TalonFX[] { this.rightLeader, this.rightFollower, this.leftLeader, this.leftFollower }) {
			// Reset settings for safety
			fx.configFactoryDefault();

			// Sets voltage compensation to 10, used for percent output
			fx.configVoltageCompSaturation(10);
			fx.enableVoltageCompensation(true);

			// Setting just in case
			fx.configNominalOutputForward(0);
			fx.configNominalOutputReverse(0);
			fx.configPeakOutputForward(1);
			fx.configPeakOutputReverse(-1);

			// TODO: fix this once weight is added
			fx.configOpenloopRamp(0.4);

			// Setting deadband(area required to start moving the motor) to 1%
			fx.configNeutralDeadband(0.01);

			// Set to coast mode, will not brake the motor when no power is sent
			fx.setNeutralMode(NeutralMode.Brake);

			/**
			 * Setting input side current limit (amps)
			 * 45 continious, 80 peak, 30 millieseconds allowed at peak
			 * 40 amp breaker can support above 40 amps for a little bit
			 * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds
			 * should be fine
			 */
			fx.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

			// Either using the integrated Falcon sensor or an external one, will change if
			// needed
			fx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		}

		// New Talon FX inverts. Would replace InvertType.InvertMotorOutput
		// this.leftLeader.setInverted(TalonFXInvertType.CounterClockwise);
		// this.rightLeader.setInverted(TalonFXInvertType.Clockwise);

		// Setting followers, followers don't automatically follow the Leader's inverts
		// so you must set the invert type to Follow the Leader
		this.rightFollower.setInverted(InvertType.FollowMaster);
		this.leftFollower.setInverted(InvertType.FollowMaster);

		this.rightFollower.follow(this.rightLeader, FollowerType.PercentOutput);
		this.leftFollower.follow(this.leftLeader, FollowerType.PercentOutput);

		this.leftLeader.setInverted(InvertType.InvertMotorOutput);
	}

	public void setWheelPIDF() {

        // set the PID values for each individual wheel
        for(TalonFX fx : new TalonFX[] {rightLeader, leftLeader}) {

            fx.config_kP(0, AutoConstants.GainsAuto.P, 0);
            fx.config_kI(0, AutoConstants.GainsAuto.I, 0);
            fx.config_kD(0, AutoConstants.GainsAuto.D, 0);
            fx.config_kF(0, AutoConstants.GainsAuto.F, 0);
            // m_talonsMaster.config_IntegralZone(0, 30);
        }
    }

	// -----------------------------------------------------------
	// Control Input
	// -----------------------------------------------------------
	public void halt() {
		this.tankDriveVolts(0, 0);
	}

	public void enableMotorSafety() {
		this.diffDrive.setSafetyEnabled(true);
	}

	public void disableMotorSafety() {
		this.diffDrive.setSafetyEnabled(false);
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		this.rightLeader.set(ControlMode.PercentOutput, leftVolts / 12);
		this.leftLeader.set(ControlMode.PercentOutput, rightVolts / 12);

		// Feed motor safety to assert that we're in control
		this.diffDrive.feed();
	}

	public void zeroGyro() {
		this.pigeon.setYaw(0);
		this.pigeon.reset();
	}

	public void resetEncoders() {
		this.rightLeader.setSelectedSensorPosition(0);
		this.leftLeader.setSelectedSensorPosition(0);
	}

	public void resetOdometry(Pose2d pose) {
		this.resetEncoders();
		this.odometry.resetPosition(this.read2dRotation(), 0, 0, pose);
	}

//
	public void setOutputMetersPerSecond(double rightMetersPerSecond, double leftMetersPerSecond) {
		// Calculate feedforward for the left and right wheels.
		double leftFeedForward = this.feedForwardL.calculate(leftMetersPerSecond);
		double rightFeedForward = this.feedForwardR.calculate(rightMetersPerSecond);

		//SmartDashboard.putNumber("left meters per sec", leftMetersPerSecond);
		//SmartDashboard.putNumber("right meters per sec", rightMetersPerSecond);

		// Convert meters per second to encoder ticks per second
		double leftVelocityTicksPerSec = metersToEncoderTicks(leftMetersPerSecond);
		double rightVelocityTicksPerSec = metersToEncoderTicks(rightMetersPerSecond);

		//SmartDashboard.putNumber("Velocity ticks per second Left", leftVelocityTicksPerSec);
		//SmartDashboard.putNumber("Velocity ticks per second Right", rightVelocityTicksPerSec);

		//SmartDashboard.putNumber("FeedForward Left", leftFeedForward);
		//SmartDashboard.putNumber("FeedForward Right", rightFeedForward);

		this.rightLeader.set(ControlMode.Velocity,
				leftVelocityTicksPerSec / 10.0,
				DemandType.ArbitraryFeedForward,
				leftFeedForward / AutoConstants.maxVolts);
		this.leftLeader.set(ControlMode.Velocity,
				rightVelocityTicksPerSec / 10.0,
				DemandType.ArbitraryFeedForward,
				rightFeedForward / AutoConstants.maxVolts);

		this.diffDrive.feed();
	}

	public void turnPower(double power) {
		leftLeader.setVoltage(power);
		rightLeader.setVoltage(-power);
	}

	// public void setSpeedMultiplier(double multiplier){
	// 	speedMultiplier = multiplier;
	// }

	// public void setManualMultiplierOn(boolean multiplierOn){
	// 	speedMultiplierOn = multiplierOn;
	// }

	public void setCoastMode(){
		rightLeader.setNeutralMode(NeutralMode.Coast);
		leftLeader.setNeutralMode(NeutralMode.Coast);
		rightFollower.setNeutralMode(NeutralMode.Coast);
		leftFollower.setNeutralMode(NeutralMode.Coast);
	}

	public void setBrakeMode(){
		rightLeader.setNeutralMode(NeutralMode.Brake);
		leftLeader.setNeutralMode(NeutralMode.Brake);
		rightFollower.setNeutralMode(NeutralMode.Brake);
		leftFollower.setNeutralMode(NeutralMode.Brake);
	}

	// -----------------------------------------------------------
	// System State
	// -----------------------------------------------------------
	public double motorRotationsToWheelRotations(double motorRotations, Transmission.GearState gearState) {
		if(gearState == Transmission.GearState.HIGH) {
			return motorRotations / (DrivetrainConstants.encoderCPR * DrivetrainConstants.highGearRatio);
		} else {
			return motorRotations / (DrivetrainConstants.encoderCPR * DrivetrainConstants.lowGearRatio);
		}
	}

	public double wheelRotationsToMeters(double wheelRotations) {
		return DrivetrainConstants.wheelDiameterMeters * Math.PI * wheelRotations;
	}

	// Encoder ticks to meters
	public double encoderTicksToMeters(double encoderTicks) {
		return this.wheelRotationsToMeters(this.motorRotationsToWheelRotations(encoderTicks, Robot.instance.robotContainer.transmission.getGearState()));
	}

	public double getLeftDistanceMeters() {
		return this.encoderTicksToMeters(this.rightLeader.getSelectedSensorPosition());
	}

	public double getRightDistanceMeters() {
		return this.encoderTicksToMeters(this.leftLeader.getSelectedSensorPosition());
	}

	public double getAvgDistanceMeters() {
		return (this.getLeftDistanceMeters() + this.getRightDistanceMeters()) / 2;
	}

	public double[] readGyro() {
		double[] angle = new double[3];
		this.pigeon.getYawPitchRoll(angle);
		return angle;
	}

	/**
	 *
	 * @return pose from encoders
	 */
	public Pose2d getEncoderPose() {
		return this.odometry.getPoseMeters();
	}

	/**
	 * If the robot is in simulation then a default pose is returned
	 *
	 * @return pose using encoders and limelight
	 */
	public Pose2d getEstimatedPose() {
		if(RobotBase.isReal()) {
			return this.poseEstimator.getEstimatedPosition();
		} else if(Timer.getFPGATimestamp() > 0.5) {
			return new Pose2d(5.0,4.0, new Rotation2d(3.1));
		}
		else {
			return new Pose2d(0.0,0.0, new Rotation2d());
		}
	}

	public Rotation2d readYawRot() {
		return Rotation2d.fromDegrees(this.readYaw());
	}

	public double readYaw() {
		return this.readGyro()[0];
	}

	public double readPitch() {
		return -this.readGyro()[1];
	}

	public double readRoll() {
		return -this.readGyro()[2];
	}

	public Rotation2d read2dRotation() {
		return Rotation2d.fromDegrees(this.readYaw());
	}

	public Pose2d getPose() {
		return this.odometry.getPoseMeters();
	}

	public double getMotorOutput() {
		return this.leftLeader.getMotorOutputVoltage();
	}

	public double metersToWheelRotations(double metersPerSecond) {
		return metersPerSecond / (DrivetrainConstants.wheelDiameterMeters * Math.PI);
	}

	public double wheelRotationsToEncoderTicks(double wheelRotations, Transmission.GearState gearState) {
		if(gearState == Transmission.GearState.HIGH) {
			return wheelRotations * DrivetrainConstants.encoderCPR * DrivetrainConstants.highGearRatio;
		}
		return wheelRotations * DrivetrainConstants.encoderCPR * DrivetrainConstants.lowGearRatio;
	}

	public double metersToEncoderTicks(double metersPerSecond) {
		GearState gearState = Transmission.instance.getGearState();
		double encoderTicks = this.wheelRotationsToEncoderTicks(
			this.metersToWheelRotations(metersPerSecond),
			gearState
		);
		return encoderTicks;
	}

	public double getHeading() {
		return this.pigeon.getYaw();
	}

	// Robot transform in 3D field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
	// using "botpose"
	public Pose3d getLimelightPose3d() {
		return this.limelight.getPose3d();
	}

	// Robot transform in 2D field-space. Translation (X,Y) Rotation(Z)
	// using "botpose"
	public Pose2d getLimelightPose2d() {
		return this.limelight.getPose2d();
	}

	// Robot transform in field-space with the alliance driverstation at the origin
	// using botpose_wpired and botpose_wpiblue
	public Pose2d getLimelightPoseRelative() {
		if(RobotBase.isReal()) {
			if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
				return this.limelight.getRedPose2d();
			} else {
				return this.limelight.getBluePose2d();
			}
		} else {
			// In simulation we just return the encoder pose.
			return this.getEncoderPose();
		}
	}

	public Pose2d getLimelightPoseBlue(){
		return this.limelight.getBluePose2d();
	}

  	/**
	 * Returns if current robot estimated pose is left or right of the center
	 * of the Charging Station taking the team alliance into account.
	 *
	 * @return Is robot left or right of the center of the Charging Station
	 */
	public boolean isLeftOfChargingStation() {
		if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
			return this.getEstimatedPose().getY() >= FieldConstants.Community.chargingStationCenterY;
		} else {
			return this.getEstimatedPose().getY() <= FieldConstants.Community.chargingStationCenterY;
		}
	}

	public boolean isRightOfChargingStation() {
		return !this.isLeftOfChargingStation();
	}

	/**
	 * Gets the angle of the target relative to the robot
	 * @return offset angle between target and the robot
	 */
	public double getTargetHorizontalOffset() {
		return this.limelight.getHorizontalOffset();
	}

	public double getTargetVerticalOffset() {
		if(this.limelight.getVerticalOffset() != 0) {
			this.offset = this.limelight.getVerticalOffset();
		}
		return this.filterVertical.calculate(this.offset);
	}

	public boolean hasValidLimelightTarget() {
		if(RobotBase.isReal()) {
			return this.limelight.getHasValidTargets();
		} else {
			return this.getHasValidTargetsSim();
		}
	}

	public int getAprilTagID() {
		if(RobotBase.isReal()) {
			return this.limelight.getAprilTagID();
		} else {
			return this.getAprilTagIDSim();
		}
	}

	// -----------------------------------------------------------
	// Processing
	// -----------------------------------------------------------
	@Override
	public void periodic() {
		//if limelight sees april tags, use limelight odometry, otherwise update from pigeon and encoders
		// if(limelight.getHasValidTargets() == 1) {
		// 	updateOdometryFromLimelight();
		// } else {
		// 	  odometry.update(readYawRot(), getLeftDistanceMeters(), getRightDistanceMeters());
		// }


		// TODO test?
		if (m_arm.armIsOut()){
			this.diffDrive.setMaxOutput(.6);
		} else {
			this.diffDrive.setMaxOutput(1);
		}

		

		this.odometry.update(this.readYawRot(), this.getLeftDistanceMeters(), this.getRightDistanceMeters());
		this.poseEstimator.update(this.readYawRot(), this.getLeftDistanceMeters(), this.getRightDistanceMeters());
		if(this.limelight.getHasValidTargets()) {
			this.poseEstimator.addVisionMeasurement(this.getLimelightPose2d(), Timer.getFPGATimestamp() - 0.3);
		}
		
		NeutralMode neutralMode = (Robot.instance.isAutonomousEnabled() || this.brakeOverride || Robot.instance.robotContainer.driverOI.getReductFactor() < 0.4) ? NeutralMode.Brake : NeutralMode.Coast;

		SmartDashboard.putString("Neutral Mode", neutralMode.name());

		for(WPI_TalonFX fx : new WPI_TalonFX[] { this.leftLeader, this.leftFollower, this.rightLeader, this.rightFollower }) {
			fx.setNeutralMode(neutralMode);
		}

		this.publishTelemetry();
	}

	public void publishTelemetry() {
		//SmartDashboard.putNumber("Odometry X", this.odometry.getPoseMeters().getX());
		//SmartDashboard.putNumber("Odometry Y", this.odometry.getPoseMeters().getY());
		//SmartDashboard.putNumber("Odometry Theta", this.odometry.getPoseMeters().getRotation().getDegrees());
		//SmartDashboard.putNumber("left encoder", this.rightLeader.getSelectedSensorPosition());
		//SmartDashboard.putNumber("right encoder", this.leftLeader.getSelectedSensorPosition());

		//SmartDashboard.putNumber("Limelight X", this.getLimelightPoseRelative().getX());
		//SmartDashboard.putNumber("Limelight Y", this.getLimelightPoseRelative().getY());
		//SmartDashboard.putNumber("Limelight Deg.", this.getLimelightPoseRelative().getRotation().getDegrees());
		//SmartDashboard.putNumber("Limelight Rad.", this.getLimelightPoseRelative().getRotation().getRadians());

		this.field2d.setRobotPose(this.getEncoderPose());
		this.fieldEstimated.setRobotPose(this.getEstimatedPose());
		this.fieldLimelight.setRobotPose(this.getLimelightPoseRelative());

		SmartDashboard.putNumber("April Tag", this.getAprilTagID());

		// SmartDashboard.putNumber("motor output", this.getMotorOutput());
		// SmartDashboard.putNumber("right enoder ticks", this.rightLeader.getSelectedSensorPosition());
		// SmartDashboard.putNumber("left enoder ticks", this.leftLeader.getSelectedSensorPosition());
		// SmartDashboard.putNumber("poseX", this.getEncoderPose().getX());
		// SmartDashboard.putNumber("botposeX", (this.limelight.getPose()[0] - DrivetrainConstants.xOffsetField));
	}

	// ----------------------------------------------------
	// Simulation
	// ----------------------------------------------------

	public void simulationInit() {
		// PhysicsSim.getInstance().addTalonFX(rightFollower, 0.75, 20660);
		// PhysicsSim.getInstance().addTalonFX(leftFollower, 0.75, 20660);
	}

	@Override
    public void simulationPeriodic() {
		// PhysicsSim.getInstance().run();
        this.driveSim.run();
    }

	public boolean getHasValidTargetsSim() {
		double heading = this.getEncoderPose().getRotation().getDegrees();

		if(DriverStation.getAlliance() == DriverStation.Alliance.Red) return heading < 75 || heading > -75;
		else return heading > 135 || heading < -135;
	}

	public int getAprilTagIDSim() {
		return 8;
	}
}
