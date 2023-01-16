// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class Drivetrain extends SubsystemBase {
  public final WPI_TalonFX m_leftLeader = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainLeftBackTalonFX);
  public final WPI_TalonFX m_rightLeader = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainRightBackTalonFX);
  public final WPI_TalonFX m_leftFollower = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainLeftFrontTalonFX);
  public final WPI_TalonFX m_rightFollower = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainRightFrontTalonFX);

  private Supplier<Transmission.GearState> m_gearStateSupplier;

  public DifferentialDrive m_diffDrive;

  // Set up the BuiltInAccelerometer
  public WPI_PigeonIMU m_pigeon = new WPI_PigeonIMU(Constants.CANBusIDs.kPigeonIMU);

  //Drivetrain kinematics, feed it width between wheels
  private SimpleMotorFeedforward m_feedForward;

  //Drivetrain odometry to keep track of our position on the field
  private DifferentialDriveOdometry m_odometry;

  private final Field2d m_field2d = new Field2d();

  private double m_yaw;

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------

  /** Creates a new Drivetrain. */
  public Drivetrain(Supplier<Transmission.GearState> gearStateSupplier) {
    m_gearStateSupplier = gearStateSupplier;

    // Motors
    configmotors();

    // Configure PID values for the talons
    setWheelPIDF();

    m_diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    m_feedForward = DrivetrainConstants.kFeedForward;

    // Setup odometry to start at position 0,0 (top left of field)
    m_yaw = m_pigeon.getYaw();
    SmartDashboard.putNumber("Initial robot yaw", m_yaw);

    Rotation2d initialHeading = new Rotation2d(m_yaw);

    // Reset encoders and gyro
    resetEncoders();
    zeroGyro();

    // Start with default Pose2d(0, 0, 0)
    m_odometry = new DifferentialDriveOdometry(initialHeading, 0, 0);

    m_field2d.setRobotPose(getPose());
    SmartDashboard.putData("Field", m_field2d);
  }

  public void setWheelPIDF() {
    // set the PID values for each individual wheel
    for (TalonFX fx : new TalonFX[] { m_leftLeader, m_rightLeader }) {
      fx.config_kP(0, DrivetrainConstants.GainsBalance.P, 0);
      fx.config_kI(0, DrivetrainConstants.GainsBalance.I, 0);
      fx.config_kD(0, DrivetrainConstants.GainsBalance.D, 0);
      fx.config_kF(0, DrivetrainConstants.GainsBalance.F, 0);
      // m_talonsMaster.config_IntegralZone(0, 30);
    }
  }

  public void configmotors() { // new
    // Configure the motors
    for (TalonFX fx : new TalonFX[] { m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower }) {
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
    // m_leftLeader.setInverted(TalonFXInvertType.CounterClockwise);
    // m_rightLeader.setInverted(TalonFXInvertType.Clockwise);

    // Setting followers, followers don't automatically follow the Leader's inverts
    // so you must set the invert type to Follow the Leader
    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_rightFollower.setInverted(InvertType.FollowMaster);

    m_leftFollower.follow(m_leftLeader, FollowerType.PercentOutput);
    m_rightFollower.follow(m_rightLeader, FollowerType.PercentOutput);

    m_rightLeader.setInverted(InvertType.InvertMotorOutput);
  }

  // Revert back to the integrated sensors (encoders)
  public void setIntegratedSensors() {
    // Configure the motors
    for (TalonFX fx : new TalonFX[] { m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower }) {
      // Use 1-to-1 coefficient for the encoders.
      fx.configSelectedFeedbackCoefficient(1);
      fx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
  }

  // Use Gyro Pitch as the sensor
  public void setPIDPigeonSensors() {
    // Configure the motors
    for (TalonFX fx : new TalonFX[] { m_leftLeader, m_rightLeader }) {

      // Configure the RemoteSensor0 and set it to the Pigeon pitch source
      fx.configRemoteFeedbackFilter(m_pigeon.getDeviceID(),
          RemoteSensorSource.Pigeon_Pitch,
          0);
      fx.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

      // Convert Pitch to tenths of a degree
      fx.configSelectedFeedbackCoefficient(3600.0 / DrivetrainConstants.kUnitsPerRevolution);
    }
  }

  public void setPIDSlot(int slot) {
    int PID_PRIMARY = 0;
    m_leftLeader.selectProfileSlot(slot, PID_PRIMARY);
    m_rightLeader.selectProfileSlot(slot, PID_PRIMARY);
  }

  // Controls to the setpoint using the internal feedback sensor
  public void setPIDSetpoint(double setpoint) {
    this.m_leftLeader.set(ControlMode.Position, setpoint);
    this.m_rightLeader.set(ControlMode.Position, setpoint);
    this.m_diffDrive.feed();
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void halt() {
    m_diffDrive.arcadeDrive(0, 0);
  }

  public void enableMotorSafety(){
    m_diffDrive.setSafetyEnabled(true);
  }

  public void disableMotorSafety(){
    m_diffDrive.setSafetyEnabled(false);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLeader.set(ControlMode.PercentOutput, leftVolts / 12);
    m_rightLeader.set(ControlMode.PercentOutput, rightVolts / 12);
    m_diffDrive.feed();
  }

  public void zeroGyro() {
    m_pigeon.reset();
  }

  public void resetEncoders() {
    m_leftLeader.setSelectedSensorPosition(0);
    m_rightLeader.setSelectedSensorPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(getRotation(), 0, 0, pose);       
  }

  public void stopDrivetrain() {
    tankDriveVolts(0.0, 0.0);
  }

  public void setOutputMetersPerSecond(double rightMetersPerSecond, double leftMetersPerSecond) {
        
    System.out.println("right m/s" + rightMetersPerSecond);
    // Calculate feedforward for the left and right wheels.
    double leftFeedForward = m_feedForward.calculate(leftMetersPerSecond);
    double rightFeedForward = m_feedForward.calculate(rightMetersPerSecond);

    SmartDashboard.putNumber("left meters per sec", leftMetersPerSecond);
    SmartDashboard.putNumber("right meters per sec", rightMetersPerSecond);

    //test comment 10/16 for auto crash
    // System.out.println("right" + rightFeedForward); 
    // System.out.println("left" + leftFeedForward);
    // m_rightFFEntry.setDouble(rightFeedForward);
    // m_leftFFEntry.setDouble(leftFeedForward);
    
    // Convert meters per second to encoder ticks per second
    var gearState = m_gearStateSupplier.get();
    double leftVelocityTicksPerSec = wheelRotationsToEncoderTicks(metersToWheelRotations(leftMetersPerSecond), gearState);
    double rightVelocityTicksPerSec = wheelRotationsToEncoderTicks(metersToWheelRotations(rightMetersPerSecond), gearState);

    SmartDashboard.putNumber("left velocity ticks per second", leftVelocityTicksPerSec);
    SmartDashboard.putNumber("right velocity ticks per second", rightVelocityTicksPerSec);

    m_leftLeader.set(ControlMode.Velocity, 
                    leftVelocityTicksPerSec/10.0, 
                    DemandType.ArbitraryFeedForward, 
                    leftFeedForward / DrivetrainConstants.k_MaxVolts);
    m_rightLeader.set(ControlMode.Velocity, 
                    rightVelocityTicksPerSec/10.0, 
                    DemandType.ArbitraryFeedForward, 
                    rightFeedForward / DrivetrainConstants.k_MaxVolts);

    m_diffDrive.feed();
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
    GearState gearState = m_gearStateSupplier.get();
    return wheelRotationsToMeters(motorRotationsToWheelRotations(encoderTicks, gearState));
  }

  public double getLeftDistanceMeters() {
    return encoderTicksToMeters(m_leftLeader.getSelectedSensorPosition());
  }

  public double getRightDistanceMeters() {
    return encoderTicksToMeters(m_rightLeader.getSelectedSensorPosition());
  }

  public double getAvgDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2;
  }

  public double[] readGyro() {
    double[] angle = new double[3];
    m_pigeon.getYawPitchRoll(angle);
    return angle;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Rotation2d getRotation(){
    
        return (Rotation2d.fromDegrees(m_yaw));      
  }
  
  public double getMotorOutput(){
    return m_rightLeader.getMotorOutputVoltage();
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

  public double getHeading(){
    return m_pigeon.getFusedHeading();
  }

// ----------------------------------------------------
// Process Logic
// ----------------------------------------------------

// This method will be called once per scheduler run
  @Override
  public void periodic() {
    

   m_odometry.update(getRotation(), getLeftDistanceMeters(), getRightDistanceMeters());
    publishTelemetry();

  }

  public void publishTelemetry(){
    SmartDashboard.putNumber("motor output", getMotorOutput());
    m_field2d.setRobotPose(getPose());
    SmartDashboard.putNumber("right enoder ticks", m_rightLeader.getSelectedSensorPosition());
    SmartDashboard.putNumber("left enoder ticks", m_leftLeader.getSelectedSensorPosition());
  }

}
