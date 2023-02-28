package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

/*/

Bottom:
	- Soft: 13500
	- Hard: 15000
Top:
	- Soft: -11596
	- Hard: 

/*/

public class Elevator extends SubsystemBase {
	// Fwd: Down
	// Rev: Up
	public final WPI_TalonFX motor = new WPI_TalonFX(Constants.CANBusIDs.ElevatorTalon1);

	// True: Unlocked
	// False: Locked
	public final Solenoid lockingPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticIDs.kElevatorLock);

	private ShuffleboardTab tab;
	private GenericEntry entryPower, entryPosition;
	private GenericEntry entryTopLimit, entryHomeLimit;

	// ------------ Initialization -----------------------------
  
	public Elevator() {
		this.configureMotors();
		this.lock(true);
		this.setupShuffleboard();
	}

	public void configureMotors() {
		// Configure the motors
		for(TalonFX fx : new TalonFX[] { this.motor }) {
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

		// Home limit switch. Stop motor if this switch is triggered.
		this.motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

		// Top limit switch. Read as a digital input
		this.motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

		this.motor.configForwardSoftLimitThreshold(ElevatorConstants.bottomSoftLimit);
		this.motor.configReverseSoftLimitThreshold(ElevatorConstants.topSoftLimit);

		this.motor.configForwardSoftLimitEnable(true);
		this.motor.configReverseSoftLimitEnable(true);

		this.motor.overrideSoftLimitsEnable(true);
	}


	public void setupShuffleboard() {
		this.tab = Shuffleboard.getTab("ElevatorArm");
  
		this.entryPosition = tab.add("Elevator Position", this.getPosition())
			.withPosition(3, 0)
			.getEntry();

    	this.entryPower = tab.add("Elevator Power", this.motor.getMotorOutputPercent())
			.withPosition(3, 2)
			.getEntry();

		// Limit Switches
		ShuffleboardLayout switchLayout = this.tab
			.getLayout("Elevator Limits", BuiltInLayouts.kList)
			.withSize(2, 2)
			.withPosition(4, 0);
		this.entryTopLimit = switchLayout.add("Top Limit Switch", this.limitTopClosed()).getEntry();
		this.entryHomeLimit = switchLayout.add("Home Limit Switch", this.limitHomeClosed()).getEntry();
	}

	// --------------- Control Input ---------------------

	public void halt() {
		this.lock(true);
	}

	public void control(double power) {
		SmartDashboard.putNumber("Elevator power", power);
		if(this.pastBottomLimit()) power = Math.min(power, 0.0);
		double deadbandPower = MathUtil.applyDeadband(power, 0.1);
		SmartDashboard.putNumber("Elevator deadband power", deadbandPower);

		this.lock(deadbandPower == 0);
		this.setPower(deadbandPower);
	}

	public void setPower(double power) {
		this.motor.set(ControlMode.PercentOutput, MathUtil.clamp(power, -0.2, 0.2));
	}

	public void lock(boolean shouldLock) {
		this.lockingPiston.set(!shouldLock);
		this.motor.set(ControlMode.PercentOutput, 0.0); // just to be safe
	}

	// ------------- System State -------------------

	public boolean limitTopClosed() {
		return this.motor.getSensorCollection().isRevLimitSwitchClosed() == 1;
	}

	public boolean limitHomeClosed() {
		return this.motor.getSensorCollection().isFwdLimitSwitchClosed() == 1;
	}

	public double getPosition() {
		return this.motor.getSelectedSensorPosition();
	}

	public void overrideEncoderPosition(double ticks) {
		this.motor.setSelectedSensorPosition(ticks);
	}

	public boolean pastBottomLimit() {
		return this.getPosition() >= ElevatorConstants.bottomSoftLimit;
	}

	 // ------------- Process State -------------------

	@Override
	public void periodic() {
		if(this.limitTopClosed() || this.pastBottomLimit()) {
			this.halt();
		}

		this.publishTelemetry();
	}

	public void publishTelemetry() {
		this.entryPower.setDouble(this.motor.getMotorOutputPercent());
		this.entryPosition.setDouble(this.getPosition());
		this.entryTopLimit.setBoolean(this.limitTopClosed());
		this.entryHomeLimit.setBoolean(this.limitHomeClosed());
	}
}
