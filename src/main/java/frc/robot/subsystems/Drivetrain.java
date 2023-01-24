// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
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
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class Drivetrain extends SubsystemBase {
	public final WPI_TalonFX leftLeader = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainLeftBackTalonFX);
	public final WPI_TalonFX rightLeader = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainRightBackTalonFX);
	public final WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainLeftFrontTalonFX);
	public final WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainRightFrontTalonFX);

  private final Limelight m_limelight = new Limelight();

	private Supplier<Transmission.GearState> gearStateSupplier;

	public DifferentialDrive diffDrive;

	// Set up the BuiltInAccelerometer
	public WPI_PigeonIMU pigeon = new WPI_PigeonIMU(Constants.CANBusIDs.kPigeonIMU);

	// Drivetrain kinematics, feed it width between wheels
	private SimpleMotorFeedforward feedForward;

	// Drivetrain odometry to keep track of our position on the field
	private DifferentialDriveOdometry odometry;

	private final Field2d field2d = new Field2d();

	private double yaw;

	// -----------------------------------------------------------
	// Initialization
	// -----------------------------------------------------------

	/** Creates a new Drivetrain. */
	public Drivetrain(Supplier<Transmission.GearState> gearStateSupplier) {
		this.gearStateSupplier = gearStateSupplier;

		// Configure Talon motors
		this.configureMotors();

		this.diffDrive = new DifferentialDrive(leftLeader, rightLeader);

		this.feedForward = DrivetrainConstants.kFeedForward;

		this.yaw = readGyro()[0];

		this.resetEncoders();
		this.zeroGyro();

		// Start with default Pose2d(0, 0, 0)
		this.odometry = new DifferentialDriveOdometry(new Rotation2d(yaw), 0, 0);

		this.field2d.setRobotPose(getPose());
		SmartDashboard.putData("Field", this.field2d);
	}

	public void configureMotors() {
		// Configure the motors
		for (TalonFX fx : new TalonFX[] { this.leftLeader, this.leftFollower, this.rightLeader, this.rightFollower }) {
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

			fx.configOpenloopRamp(0.1);

			// Setting deadband(area required to start moving the motor) to 1%
			fx.configNeutralDeadband(0.01);

			// Set to brake mode, will brake the motor when no power is sent
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
		this.leftFollower.setInverted(InvertType.FollowMaster);
		this.rightFollower.setInverted(InvertType.FollowMaster);

		this.leftFollower.follow(this.leftLeader, FollowerType.PercentOutput);
		this.rightFollower.follow(this.rightLeader, FollowerType.PercentOutput);

		this.rightLeader.setInverted(InvertType.InvertMotorOutput);
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
		this.leftLeader.set(ControlMode.PercentOutput, leftVolts / 12);
		this.rightLeader.set(ControlMode.PercentOutput, rightVolts / 12);

		// Feed motor safety to assert that we're in control
		this.diffDrive.feed();
	}

	public void zeroGyro() {
		this.pigeon.reset();
	}

	public void resetEncoders() {
		this.leftLeader.setSelectedSensorPosition(0);
		this.rightLeader.setSelectedSensorPosition(0);
	}

	public void resetOdometry(Pose2d pose) {
		this.resetEncoders();
		this.odometry.resetPosition(this.readYaw(), 0, 0, pose);
	}

  public void updateOdometryFromLimelight(){
    this.resetEncoders();
    this.odometry.resetPosition(this.readYaw(), 0, 0, getLimelightPose());
  }

	public void setOutputMetersPerSecond(double rightMetersPerSecond, double leftMetersPerSecond) {
		// System.out.println("right m/s" + rightMetersPerSecond);
		// Calculate feedforward for the left and right wheels.
		double leftFeedForward = this.feedForward.calculate(leftMetersPerSecond);
		double rightFeedForward = this.feedForward.calculate(rightMetersPerSecond);

		SmartDashboard.putNumber("left meters per sec", leftMetersPerSecond);
		SmartDashboard.putNumber("right meters per sec", rightMetersPerSecond);

		// test comment 10/16 for auto crash
		// System.out.println("right" + rightFeedForward);
		// System.out.println("left" + leftFeedForward);
		// this.rightFFEntry.setDouble(rightFeedForward);
		// this.leftFFEntry.setDouble(leftFeedForward);

		// Convert meters per second to encoder ticks per second
		GearState gearState = this.gearStateSupplier.get();
		double leftVelocityTicksPerSec = this.wheelRotationsToEncoderTicks(
			this.metersToWheelRotations(leftMetersPerSecond),
			gearState
		);
		double rightVelocityTicksPerSec = this.wheelRotationsToEncoderTicks(
			this.metersToWheelRotations(rightMetersPerSecond),
			gearState
		);

		SmartDashboard.putNumber("left velocity ticks per second", leftVelocityTicksPerSec);
		SmartDashboard.putNumber("right velocity ticks per second", rightVelocityTicksPerSec);

		this.leftLeader.set(ControlMode.Velocity,
				leftVelocityTicksPerSec / 10.0,
				DemandType.ArbitraryFeedForward,
				leftFeedForward / DrivetrainConstants.k_MaxVolts);
		this.rightLeader.set(ControlMode.Velocity,
				rightVelocityTicksPerSec / 10.0,
				DemandType.ArbitraryFeedForward,
				rightFeedForward / DrivetrainConstants.k_MaxVolts);

		this.diffDrive.feed();
	}

	// -----------------------------------------------------------
	// System State
	// -----------------------------------------------------------
	public double motorRotationsToWheelRotations(double motorRotations, Transmission.GearState gearState) {
		if (gearState == Transmission.GearState.HIGH) {
			return motorRotations / (DrivetrainConstants.encoderCPR * DrivetrainConstants.highGearRatio);
		} else {
			return motorRotations / (DrivetrainConstants.encoderCPR * DrivetrainConstants.lowGearRatio);
		}
	}

	public double wheelRotationsToMeters(double wheelRotations) {
		return DrivetrainConstants.kWheelDiameterMeters * Math.PI * wheelRotations;
	}

	// Encoder ticks to meters
	public double encoderTicksToMeters(double encoderTicks) {
		GearState gearState = gearStateSupplier.get();
		return wheelRotationsToMeters(motorRotationsToWheelRotations(encoderTicks, gearState));
	}

	public double getLeftDistanceMeters() {
		return encoderTicksToMeters(leftLeader.getSelectedSensorPosition());
	}

	public double getRightDistanceMeters() {
		return encoderTicksToMeters(rightLeader.getSelectedSensorPosition());
	}

	public double getAvgDistanceMeters() {
		return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2;
	}

	public double[] readGyro() {
		double[] angle = new double[3];
		pigeon.getYawPitchRoll(angle);
		return angle;
	}

  /**
   * 
   * @return pose from encoders
   */
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public Rotation2d readYaw() {

		yaw = readGyro()[0];

		return (Rotation2d.fromDegrees(yaw));
	}

	public double getMotorOutput() {
		return rightLeader.getMotorOutputVoltage();
	}

	public double metersToWheelRotations(double metersPerSecond) {
		return metersPerSecond / (DrivetrainConstants.kWheelDiameterMeters * Math.PI);
	}

	public double wheelRotationsToEncoderTicks(double wheelRotations, Transmission.GearState gearState) {
		if (gearState == Transmission.GearState.HIGH) {
			return wheelRotations * DrivetrainConstants.encoderCPR * DrivetrainConstants.highGearRatio;
		}
		return wheelRotations * DrivetrainConstants.encoderCPR * DrivetrainConstants.lowGearRatio;
	}

	public double getHeading() {
		return pigeon.getFusedHeading();
	}

  /**
   * calculated using meters
   * @return pose from the limelight
   */
  public Pose2d getLimelightPose(){

    Rotation2d rotation = new Rotation2d(m_limelight.getPose()[5] / 180 * Math.PI);
    return new Pose2d(m_limelight.getPose()[0] + DrivetrainConstants.xOffsetField, 
		m_limelight.getPose()[1] + DrivetrainConstants.yOffsetField, rotation);
  }

  /**
   * 
   * @param startPose pose where robot starts
   * @param endPose pose where robot should end
   * @param direction 0 for left, 1 for right, 2 for robot to decide
   * @return
   */
  public Trajectory navigateToDropoff(Pose2d endPose, int direction){

	Trajectory trajectory;
	Pose2d startPose;

	//if limelight has valid targets, sets start pose to limelight's pose, otherwise use odometry pose
	if (m_limelight.getHasValidTargets() == 1){
		startPose = getLimelightPose();
	} else {
		startPose = getPose();
	}

	DriverStation.Alliance color = DriverStation.getAlliance();
	
	if(color == DriverStation.Alliance.Red){
		// for red, left and right
		//if direction is specified left, or direction is unspecified and Y is on left side of field...
		if(direction == 0 || ((direction == 2 ) && (startPose.getY() <= (DrivetrainConstants.fieldWidthYMeters / 2)))){
			trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
			List.of(DrivetrainConstants.leftRedWaypoint1, DrivetrainConstants.leftRedWaypoint2), 
			endPose, DrivetrainConstants.kTrajectoryConfig);
		} else {
			trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
				List.of(DrivetrainConstants.rightRedWaypoint1, DrivetrainConstants.rightRedWaypoint2), 
				endPose, DrivetrainConstants.kTrajectoryConfig);
		}
	} else {
		// for blue, left and right
		if(direction == 0 || ((direction == 2 ) && (startPose.getY() >= (DrivetrainConstants.fieldWidthYMeters / 2)))){
			trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
				List.of(DrivetrainConstants.leftBlueWaypoint1, DrivetrainConstants.leftBlueWaypoint2), 
				endPose, DrivetrainConstants.kTrajectoryConfig);
		} else {
			trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
				List.of(DrivetrainConstants.rightBlueWaypoint1, DrivetrainConstants.rightBlueWaypoint2), 
				endPose, DrivetrainConstants.kTrajectoryConfig);
		}
	}


	return trajectory;
  }

	// ----------------------------------------------------
	// Process Logic
	// ----------------------------------------------------

	// This method will be called once per scheduler run
	@Override
	public void periodic() {

    //if limelight sees april tags, use limelight odometry, otherwise update from pigeon and encoders
	 //if((getLimelightPose().getX() != DrivetrainConstants.xOffsetField) || (getLimelightPose().getY() != DrivetrainConstants.yOffsetField)){
    if (m_limelight.getHasValidTargets() == 1){
		updateOdometryFromLimelight();
    } else {
		  odometry.update(readYaw(), getLeftDistanceMeters(), getRightDistanceMeters());
    }

		publishTelemetry();
	}

	public void publishTelemetry() {
		SmartDashboard.putNumber("motor output", getMotorOutput());
		field2d.setRobotPose(getPose());
		SmartDashboard.putNumber("right enoder ticks", rightLeader.getSelectedSensorPosition());
		SmartDashboard.putNumber("left enoder ticks", leftLeader.getSelectedSensorPosition());
    SmartDashboard.putNumber("poseX", getPose().getX());
	SmartDashboard.putNumber("botposeX", (m_limelight.getPose()[0] - DrivetrainConstants.xOffsetField));
	}
}
