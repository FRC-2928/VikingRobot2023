// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Transmission.GearState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
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

	private DifferentialDrivePoseEstimator m_poseEstimator;

	private final Field2d field2d = new Field2d();

	private double yaw;
	private final PIDController m_rollPID =
    	new PIDController(1, 0.0, 0.3);

	/* Object for simulated inputs into Talon. */
	TalonFXSimCollection m_leftDriveSim = leftLeader.getSimCollection();
	TalonFXSimCollection m_rightDriveSim = rightLeader.getSimCollection();
  
	/* Object for simulated inputs into Pigeon. */
	BasePigeonSimCollection m_pigeonSim = pigeon.getSimCollection();
	
	/* Simulation model of the drivetrain */
	DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
		DCMotor.getFalcon500(2),  //2 Falcon 500s on each side of the drivetrain.
		DrivetrainConstants.lowGearRatio,    //Low Gearing reduction.
		2.1,                //The moment of inertia of the drivetrain about its center.
		26.5,                         //Mass of the robot in kg.
		DrivetrainConstants.kWheelDiameterMeters/2,  //Robot wheel radius.
		DrivetrainConstants.kTrackWidthMeters,     //Distance between wheels.
	
		/*
		 * The standard deviations for measurement noise:
		 * x and y:          0.001 m
		 * heading:          0.001 rad
		 * l and r velocity: 0.1   m/s
		 * l and r position: 0.005 m
		 */
		null //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
	  );
	  
	/*
 	private final PIDController m_rightController =
    	new PIDController(1, 0.0, 0.3);    
	// -----------------------------------------------------------
	// Initialization
	// -----------------------------------------------------------
	*/
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
		m_poseEstimator = new DifferentialDrivePoseEstimator(DrivetrainConstants.kDriveKinematics, new Rotation2d(yaw), 0, 0, getLimelightPose());

		this.field2d.setRobotPose(getEncoderPose());
		SmartDashboard.putData("Field", this.field2d);

		m_rollPID.setTolerance(0.28);
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
	public void BalanceRollPitch(double output) {
    
		/*SmartDashboard.putNumber("Requested Velocity", velocity);
		SmartDashboard.putNumber("Setpoint Velocity", setpointVel);
		SmartDashboard.putNumber("Setpoint Position", setpointPos);
		*/
		// Send it through a PID controller
		double rollPIDVolts = m_rollPID.calculate(this.readRoll(), 0);
		//double rightPIDVolts = m_rightController.calculate(this.readRoll(), 0);
		SmartDashboard.putNumber("PID Volts", rollPIDVolts);
		//SmartDashboard.putNumber("Right PID Volts", rightPIDVolts);
		
		// Add the voltage values and send them to the motors
		if (this.readRoll() > 0)  this.tankDriveVolts(-output+rollPIDVolts, -output-rollPIDVolts);
		else this.tankDriveVolts(-output-rollPIDVolts, -output+rollPIDVolts);
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
		this.odometry.resetPosition(this.readYawRot(), 0, 0, pose);
	}

  public void updateOdometryFromLimelight(){
    this.resetEncoders();
    this.odometry.resetPosition(this.readYawRot(), 0, 0, getLimelightPose());
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
	public Pose2d getEncoderPose() {
		return odometry.getPoseMeters();
	}

	/**
	 * 
	 * @return pose using encoders and limelight
	 */
	public Pose2d getEstimatedPose(){
		return m_poseEstimator.getEstimatedPosition();
	}

	public Rotation2d readYawRot() {

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

	public double metersToEncoderTicks(double metersPerSecond) {
		GearState gearState = this.gearStateSupplier.get();
		double encoderTicks = this.wheelRotationsToEncoderTicks(
			this.metersToWheelRotations(metersPerSecond),
			gearState
		);
		return encoderTicks;
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

	// ----------------------------------------------------
	// Process Logic
	// ----------------------------------------------------

	// This method will be called once per scheduler run
	@Override
	public void periodic() {

    //if limelight sees april tags, use limelight odometry, otherwise update from pigeon and encoders
    // if (m_limelight.getHasValidTargets() == 1){
	// 	updateOdometryFromLimelight();
    // } else {
	// 	  odometry.update(readYawRot(), getLeftDistanceMeters(), getRightDistanceMeters());
    // }

		odometry.update(readYawRot(), getLeftDistanceMeters(), getRightDistanceMeters());
		m_poseEstimator.update(readYawRot(), getLeftDistanceMeters(), getRightDistanceMeters());
		if (m_limelight.getHasValidTargets() == 1){
			m_poseEstimator.addVisionMeasurement(getLimelightPose(), edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - .3);
		}

		publishTelemetry();
	}

	public void publishTelemetry() {
		SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
		SmartDashboard.putNumber("Odometry Theta", odometry.getPoseMeters().getRotation().getDegrees());
		field2d.setRobotPose(getEncoderPose());

		// SmartDashboard.putNumber("motor output", getMotorOutput());	
		// SmartDashboard.putNumber("right enoder ticks", rightLeader.getSelectedSensorPosition());
		// SmartDashboard.putNumber("left enoder ticks", leftLeader.getSelectedSensorPosition());
		// SmartDashboard.putNumber("poseX", getEncoderPose().getX());
		// SmartDashboard.putNumber("botposeX", (m_limelight.getPose()[0] - DrivetrainConstants.xOffsetField));
	}
	public double readYaw() {
		
		return this.readGyro()[0];
	}
	public double readPitch() {
		
		return this.readGyro()[2];
	}
	public double readRoll() {
		
		return this.readGyro()[1];
	}

	// ----------------------------------------------------
	// Simulation
	// ----------------------------------------------------

	@Override
	public void simulationPeriodic() {
		/* Pass the robot battery voltage to the simulated Talon FXs */
		m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
		m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

		/*
		* CTRE simulation is low-level, so SimCollection inputs
		* and outputs are not affected by SetInverted(). Only
		* the regular user-level API calls are affected.
		*
		* WPILib expects +V to be forward.
		* Positive motor output lead voltage is ccw. We observe
		* on our physical robot that this is reverse for the
		* right motor, so negate it.
		*
		* We are hard-coding the negation of the values instead of
		* using getInverted() so we can catch a possible bug in the
		* robot code where the wrong value is passed to setInverted().
		*/
		m_driveSim.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(),
							-m_rightDriveSim.getMotorOutputLeadVoltage());

		/*
		* Advance the model by 20 ms. Note that if you are running this
		* subsystem in a separate thread or have changed the nominal
		* timestep of TimedRobot, this value needs to match it.
		*/
		m_driveSim.update(0.02);

		/*
		* Update all of our sensors.
		*
		* Since WPILib's simulation class is assuming +V is forward,
		* but -V is forward for the right motor, we need to negate the
		* position reported by the simulation class. Basically, we
		* negated the input, so we need to negate the output.
		*/
		m_leftDriveSim.setIntegratedSensorRawPosition(
			(int)metersToEncoderTicks(m_driveSim.getLeftPositionMeters()));	

		m_leftDriveSim.setIntegratedSensorVelocity(
			(int)metersToEncoderTicks(m_driveSim.getLeftVelocityMetersPerSecond()) / 10);

		m_rightDriveSim.setIntegratedSensorRawPosition(
			(int)metersToEncoderTicks(-m_driveSim.getRightPositionMeters()));

		m_rightDriveSim.setIntegratedSensorVelocity(
			(int)metersToEncoderTicks(-m_driveSim.getRightVelocityMetersPerSecond()) / 10);

		m_pigeonSim.setRawHeading(-m_driveSim.getHeading().getDegrees());
	}

}
