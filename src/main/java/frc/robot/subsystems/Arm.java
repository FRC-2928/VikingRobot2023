package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANBusIDs;

public class Arm extends SubsystemBase {
	public final WPI_TalonFX motorLead = new WPI_TalonFX(Constants.CANBusIDs.ArmTalon1);
	public final WPI_TalonFX motorFollower = new WPI_TalonFX(Constants.CANBusIDs.ArmTalon2);
	public final WPI_CANCoder encoder = new WPI_CANCoder(Constants.CANBusIDs.ArmEncoder);

	private ShuffleboardTab tab;
	private GenericEntry entryPower, entryPosition;

	// True: Unlocked
	// False: Locked
	private final Solenoid lockingPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticIDs.armLock);

	// -----------------------------------------------------------
	// Initialization
	// -----------------------------------------------------------
	public Arm() {
		for(TalonFX fx : new TalonFX[] { this.motorLead, this.motorFollower}) {
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
		
			fx.configRemoteFeedbackFilter(CANBusIDs.ArmEncoder, RemoteSensorSource.CANCoder, 0, 0);
			fx.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
		}

		this.motorFollower.setInverted(InvertType.FollowMaster);
		this.motorFollower.follow(this.motorLead);

		this.setupShuffleboard();
	}

	public void setupShuffleboard() {
		this.tab = Shuffleboard.getTab("ElevatorArm");

		this.entryPosition = tab.add("Arm Position", this.getPosition())
			.withPosition(8, 0)
			.getEntry();

    	this.entryPower = tab.add("Arm Power", this.motorLead.getMotorOutputPercent())
			.withPosition(8, 2)
			.getEntry();
	}
			
	// -----------------------------------------------------------
	// Control Input
	// -----------------------------------------------------------

	public void halt() {
		this.lock(true);
	}

	public void control(double power) {
		if(this.pastTopLimit()) power = Math.min(power, 0.0);
		if(this.pastBottomLimit()) power = Math.max(power, 0.0);
		double deadbandPower = MathUtil.applyDeadband(power, 0.25);

		this.lock(deadbandPower == 0);
		this.setPower(deadbandPower);
	}

	public void setPower(double power) {
		power *= -1;
		this.motorLead.set(ControlMode.PercentOutput, MathUtil.clamp(power, -0.5, 0.5));
	}

	public void lock(boolean shouldLock) {
		this.lockingPiston.set(!shouldLock);
		this.motorLead.set(ControlMode.PercentOutput, 0.0); // just to be safe
	}

	// -----------------------------------------------------------
	// System State
	// -----------------------------------------------------------

	public double getPosition() {
		return this.encoder.getAbsolutePosition();
	}

	private boolean pastTopLimit() {
		return this.getPosition() <= ArmConstants.homeAngleLimit;
	}

	private boolean pastBottomLimit() {
		return this.getPosition() >= ArmConstants.maxAngleLimit;
	}

	/**
	 * 
	 * @return whether the arm is far enough out to limit speed - beyond pickup position
	 */
	public boolean armIsOut(){
		return (getPosition() > ArmConstants.lowPositionCone + 10);
	}

	// -----------------------------------------------------------
	// Processing
	// -----------------------------------------------------------
	@Override
	public void periodic() {
		if(this.pastBottomLimit() || this.pastTopLimit()) this.halt();

		this.entryPower.setDouble(this.motorLead.getMotorOutputPercent());
		this.entryPosition.setDouble(this.getPosition());
		// SmartDashboard.putNumber("Arm Position", getPosition());
	}
}
