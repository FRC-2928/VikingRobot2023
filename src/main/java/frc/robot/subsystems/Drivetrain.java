// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Transmission.GearState;

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
  public final WPI_TalonFX leftLeader = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainLeftBackTalonFX);
  public final WPI_TalonFX rightLeader = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainRightBackTalonFX);
  public final WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainLeftFrontTalonFX);
  public final WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainRightFrontTalonFX);

  private Supplier<Transmission.GearState> m_gearStateSupplier;

  public DifferentialDrive m_diffDrive;

  // Set up the BuiltInAccelerometer
  public WPI_PigeonIMU pigeon = new WPI_PigeonIMU(Constants.CANBusIDs.kPigeonIMU);

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

    m_diffDrive = new DifferentialDrive(leftLeader, rightLeader);

    // Reset encoders and gyro
    resetEncoders();
    zeroGyro();
  }

  public void setWheelPIDF() {
    // set the PID values for each individual wheel
    for (TalonFX fx : new TalonFX[] { leftLeader, rightLeader }) {
      fx.config_kP(0, DrivetrainConstants.GainsBalance.P, 0);
      fx.config_kI(0, DrivetrainConstants.GainsBalance.I, 0);
      fx.config_kD(0, DrivetrainConstants.GainsBalance.D, 0);
      fx.config_kF(0, DrivetrainConstants.GainsBalance.F, 0);
      // m_talonsMaster.config_IntegralZone(0, 30);
    }
  }

  public void configmotors() { // new
    // Configure the motors
    for (TalonFX fx : new TalonFX[] { leftLeader, leftFollower, rightLeader, rightFollower }) {
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
    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);

    leftFollower.follow(leftLeader, FollowerType.PercentOutput);
    rightFollower.follow(rightLeader, FollowerType.PercentOutput);

    rightLeader.setInverted(InvertType.InvertMotorOutput);
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void halt() {
    m_diffDrive.arcadeDrive(0, 0);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.set(ControlMode.PercentOutput, leftVolts / 12);
    rightLeader.set(ControlMode.PercentOutput, rightVolts / 12);
    m_diffDrive.feed();
  }

  public void zeroGyro() {
    pigeon.reset();
  }

  public void resetEncoders() {
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  public double motorRotationsToWheelRotations(double motorRotations, Transmission.GearState gearState) {
    if (gearState == Transmission.GearState.HIGH) {
      return motorRotations / (DrivetrainConstants.EncoderCPR * DrivetrainConstants.HighGearRatio);
    } else {
      return motorRotations / (DrivetrainConstants.EncoderCPR * DrivetrainConstants.LowGearRatio);
    }
  }

  public double wheelRotationsToMeters(double wheelRotations) {
    return DrivetrainConstants.WheelDiameterMeters * Math.PI * wheelRotations;
  }

  // Encoder ticks to meters
  public double encoderTicksToMeters(double encoderTicks) {
    GearState gearState = m_gearStateSupplier.get();
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

  // This method will be called once per scheduler run
  @Override
  public void periodic() {}
}
