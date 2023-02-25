// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
	public final WPI_TalonFX talonLeader = new WPI_TalonFX(Constants.CANBusIDs.ArmTalon1);
	public final WPI_TalonFX talonFollower = new WPI_TalonFX(Constants.CANBusIDs.ArmTalon2);
	public final WPI_CANCoder encoder = new WPI_CANCoder(0);

	// private final Solenoid lockPiston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIDs.kArmLock);
	
	public Arm() {
		for(TalonFX fx : new TalonFX[] { this.talonLeader, this.talonFollower}) {
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
		this.talonLeader.configRemoteFeedbackFilter(0, RemoteSensorSource.CANCoder, 0, 0);
		this.talonLeader.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

		this.talonFollower.follow(this.talonLeader);
	}

	public double getPosition() {
		return this.encoder.getAbsolutePosition();
	}

	public void setPower(double power) {
		SmartDashboard.putNumber("Arm Power", power);
		double deadbandPower = MathUtil.applyDeadband(power, 0.05);
		SmartDashboard.putNumber("Deadband Arm Power", deadbandPower);
		this.talonLeader.set(ControlMode.PercentOutput, power);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Arm Position", getPosition());
	}
}
