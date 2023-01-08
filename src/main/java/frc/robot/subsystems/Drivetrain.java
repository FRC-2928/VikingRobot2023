// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;


public class Drivetrain extends SubsystemBase {
  
  private final WPI_TalonFX m_leftLeader = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainLeftBackTalonFX);
  private final WPI_TalonFX m_rightLeader = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainRightBackTalonFX);
  private final WPI_TalonFX m_leftFollower = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainLeftFrontTalonFX);
  private final WPI_TalonFX m_rightFollower = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainRightFrontTalonFX);

  private Supplier<Transmission.GearState> m_gearStateSupplier;

  private DifferentialDrive m_diffDrive;
  
  // Set up the BuiltInAccelerometer
  private WPI_PigeonIMU m_pigeon = new WPI_PigeonIMU(Constants.CANBusIDs.kPigeonIMU);
  private double m_yaw;

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------

  /** Creates a new Drivetrain. */
  public Drivetrain(Supplier<Transmission.GearState> gearStateSupplier) {

    m_gearStateSupplier = gearStateSupplier;

    // Motors
    configmotors();

    // PID values for the talons
    setWheelPIDF();

    m_diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);   

    // new Zero the encoders and gyro
    resetEncoders();
    zeroGyro();

    m_yaw = m_pigeon.getYaw();
  }

  public void setWheelPIDF() {
    // set the PID values for each individual wheel
    for(TalonFX fx : new TalonFX[] {m_leftLeader, m_rightLeader}){
        
        fx.config_kP(0, DrivetrainConstants.kGainsProfiled.kP, 0);
        fx.config_kI(0, DrivetrainConstants.kGainsProfiled.kI, 0);
        fx.config_kD(0, DrivetrainConstants.kGainsProfiled.kD, 0);
        fx.config_kF(0, DrivetrainConstants.kGainsProfiled.kF, 0);
        // m_talonsMaster.config_IntegralZone(0, 30);
    }
  }

  public void configmotors() { //new

      // Configure the motors
      for(TalonFX fx : new TalonFX[] {m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower}){
          //Reset settings for safety
          fx.configFactoryDefault();

          //Sets voltage compensation to 10, used for percent output
          fx.configVoltageCompSaturation(10);
          fx.enableVoltageCompensation(true);

          //Setting just in case
          fx.configNominalOutputForward(0);
          fx.configNominalOutputReverse(0);
          fx.configPeakOutputForward(1);
          fx.configPeakOutputReverse(-1);

          fx.configOpenloopRamp(0.1);

          //Setting deadband(area required to start moving the motor) to 1%
          fx.configNeutralDeadband(0.01);

          //Set to brake mode, will brake the motor when no power is sent
          fx.setNeutralMode(NeutralMode.Coast);

          /** 
           * Setting input side current limit (amps)
           * 45 continious, 80 peak, 30 millieseconds allowed at peak
           * 40 amp breaker can support above 40 amps for a little bit
           * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
           */
          fx.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

          //Either using the integrated Falcon sensor or an external one, will change if needed
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

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }
  
  public void arcadeDrive(DoubleSupplier move, DoubleSupplier rotate){
    arcadeDrive(move.getAsDouble(), rotate.getAsDouble(), true);
  }

  public void arcadeDrive(double move, double rotate, boolean squaredInputs){
    m_diffDrive.arcadeDrive(move, rotate, squaredInputs);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLeader.set(ControlMode.PercentOutput, leftVolts/12);
    m_rightLeader.set(ControlMode.PercentOutput, rightVolts/12);
    m_diffDrive.feed();
  }

  public void zeroGyro(){
    m_pigeon.reset();
  }

  public void resetEncoders(){
    m_leftLeader.setSelectedSensorPosition(0);
    m_rightLeader.setSelectedSensorPosition(0);
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  public double motorRotationsToWheelRotations(double motorRotations, Transmission.GearState gearState) {
    if (gearState == Transmission.GearState.HIGH) {
        return motorRotations/(DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kHighGearRatio);
    }
    return motorRotations/(DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kLowGearRatio);
  }

  public double wheelRotationsToMeters(double wheelRotations) {
    return DrivetrainConstants.kWheelDiameterMeters * Math.PI * wheelRotations;
  }
    // Encoder ticks to meters
    public double encoderTicksToMeters(double encoderTicks) {
      var gearState = m_gearStateSupplier.get();
      double wheelRotations = motorRotationsToWheelRotations(encoderTicks, gearState);
      return wheelRotationsToMeters(wheelRotations);
  }
  
  public double getLeftDistanceMeters() {
    return encoderTicksToMeters(m_leftLeader.getSelectedSensorPosition());
  }

  public double getRightDistanceMeters() {        
      return encoderTicksToMeters(m_rightLeader.getSelectedSensorPosition());
  }

  public double getAvgDistanceMeters(){
      return (getLeftDistanceMeters() + getRightDistanceMeters()) /2;
  }
  
  public Rotation2d getRotation(){
    m_yaw = m_pigeon.getYaw();
    if (RobotBase.isReal()) {
       return (Rotation2d.fromDegrees(Math.IEEEremainder(m_yaw, 360.0d) * -1.0d)); 
    } else {
        return (Rotation2d.fromDegrees(m_yaw));
    }       
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
