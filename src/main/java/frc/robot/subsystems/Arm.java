package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANBusIDs;

public class Arm extends SubsystemBase {
	public final WPI_TalonFX motorLead = new WPI_TalonFX(Constants.CANBusIDs.ArmTalonLeader);
	public final WPI_TalonFX motorFollower = new WPI_TalonFX(Constants.CANBusIDs.ArmTalonFollower);

	public final WPI_CANCoder encoder = new WPI_CANCoder(Constants.CANBusIDs.ArmEncoder);

	// True: Unlocked
	// False: Locked
	private final Solenoid lockingPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticIDs.armLock);

	public Arm() {
		for(final WPI_TalonFX fx : new WPI_TalonFX[] { this.motorLead, this.motorFollower}) {
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

		this.motorLead.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

		Telemetry.track("Arm Position", this::getPosition, false);
		Telemetry.track("Arm Limit", () -> this.motorLead.getSensorCollection().isRevLimitSwitchClosed() == 1, false);
	}

	public void halt() {
		this.lock(true);
	}

	public void control(double power) {
		if(this.pastTopLimit()) power = Math.max(power, 0.0);
		if(this.pastBottomLimit()) power = Math.min(power, 0.0);

		final double deadbandPower = MathUtil.applyDeadband(power, 0.25);

		this.lock(deadbandPower == 0);
		this.setPower(deadbandPower);
	}

	public void setPower(final double power) {
		this.motorLead.set(ControlMode.PercentOutput, MathUtil.clamp(power, -0.5, 0.5));
	}

	public void lock(final boolean shouldLock) {
		this.lockingPiston.set(!shouldLock);
		this.motorLead.set(ControlMode.PercentOutput, 0.0); // just to be safe
	}

	public double getPosition() {
		return this.encoder.getAbsolutePosition();
	}

	private boolean pastTopLimit() {
		return this.getPosition() <= ArmConstants.homeAngleLimit;
	}

	private boolean pastBottomLimit() {
		return this.getPosition() >= ArmConstants.maxAngleLimit;
	}

	// Returns whether the arm is far enough out to limit speed - beyond pickup position
	public boolean armIsOut() {
		return (this.getPosition() > ArmConstants.lowPositionCone + 10);
	}

	@Override
	public void periodic() {
		if(this.pastBottomLimit() || this.pastTopLimit()) this.halt();

		if(this.motorLead.getSensorCollection().isRevLimitSwitchClosed() == 1) {
            //System.out.println(this.encoder.getAbsolutePosition());
			this.encoder.configMagnetOffset(this.encoder.configGetMagnetOffset() - (this.encoder.getAbsolutePosition() - ArmConstants.armLimitSwitchOffset));
		}
	}
}
