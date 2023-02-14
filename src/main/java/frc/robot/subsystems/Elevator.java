// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  public final WPI_TalonFX talon1 = new WPI_TalonFX(Constants.CANBusIDs.ElevatorTalon1);
  Solenoid m_elevatorSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIDs.kArmSolenoid);
  private boolean isFound = false;
  
  // ------------ Initialization -----------------------------
  
  /** Creates a new Elevator. */
  public Elevator() {
    configureMotors();
    setSolenoidBrake();
  }

  public void configureMotors() {
		// Configure the motors
		for(TalonFX fx : new TalonFX[] { this.talon1}) {
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
    
	}

// --------------- Control Input ---------------------

  public void setPower(double power){
	talon1.set(ControlMode.PercentOutput, power);
  }

  public void setSolenoidBrake(){
    m_elevatorSolenoid.set(false);
  }

  public void setSolenoidMove(){
    m_elevatorSolenoid.set(true);
  }

  /**
   * 
   * @param found whether the elevator knows its position
   */
  public void isFound(boolean found){
	isFound = found;
  }

  // ------------- System State -------------------

  public double getEncoderTicks() {
	return talon1.getSelectedSensorPosition();
  }

  /**
   * 
   * @return whether the elevator knows where it is
   */
  public boolean isFound(){
	return isFound;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!isFound){
      setPower(ElevatorConstants.defaultPower);
    }
  }
}
