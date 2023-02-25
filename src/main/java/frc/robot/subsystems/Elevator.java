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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  public final WPI_TalonFX talon1 = new WPI_TalonFX(Constants.CANBusIDs.ElevatorTalon1);
  Solenoid elevatorSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIDs.kElevatorLock);
  private boolean isFound = false;

  private ShuffleboardTab elevatorTab;
  private GenericEntry elevatorPowerEntry, elevatorPositionEntry;
  private GenericEntry topLimitSwitchEntry, homeLimitSwitchEntry;
  
  // ------------ Initialization -----------------------------
  
  /** Creates a new Elevator. */
  public Elevator() {
    configureMotors();
    setSolenoidBrake();
    setupShuffleboard();
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
    // Top limit switch.  Stop motor if this switch is triggered.
    talon1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
                                          LimitSwitchNormal.NormallyOpen);

    // Home limit switch. Read as a digital input
    talon1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
                                          LimitSwitchNormal.NormallyOpen);

    // talon1.configForwardSoftLimitThreshold(100);
    // talon1.configReverseSoftLimitThreshold(-100);
    // talon1.configForwardSoftLimitEnable(false);
    // talon1.configReverseSoftLimitEnable(false);
    // talon1.overrideSoftLimitsEnable(false);
	}

  public void setupShuffleboard() {
    elevatorTab = Shuffleboard.getTab("Elevator"); 

    elevatorPowerEntry = elevatorTab.add("Motor Power", talon1.getMotorOutputPercent())
      .withPosition(3, 0)
      .getEntry();  

    elevatorPositionEntry = elevatorTab.add("Elevator Position", getEncoderTicks())
      .withPosition(5, 0)
      .getEntry();    

    // Limit Switches
    ShuffleboardLayout switchLayout = Shuffleboard.getTab("Elevator")
      .getLayout("Ramp", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withPosition(8, 0); 
    topLimitSwitchEntry = switchLayout.add("Top Limit Switch", topLimitSwitchClosed()).getEntry(); 
    homeLimitSwitchEntry = switchLayout.add("Home Limit Switch", homeLimitSwitchClosed()).getEntry();  
  }

// --------------- Control Input ---------------------

  public void setPower(double power) {
    double deadbandPower = MathUtil.applyDeadband(power, 0.05);
	  talon1.set(ControlMode.PercentOutput, MathUtil.clamp(power, -0.5, 0.5));
  }

  public void setEncoderTicks(double ticks) {
    talon1.setSelectedSensorPosition(ticks);
  }

  public void setSolenoidBrake() {
    elevatorSolenoid.set(false);
  }

  public void setSolenoidMove() {
    elevatorSolenoid.set(true);
  }

  public void setBrakeEnabled() {
    talon1.overrideLimitSwitchesEnable(true);
  }

  public void setBrakeDisabled() {
    talon1.overrideLimitSwitchesEnable(false);
  }

  // ------------- System State -------------------

  public boolean topLimitSwitchClosed() {
    return talon1.getSensorCollection().isFwdLimitSwitchClosed() == 1;
  }

  public boolean homeLimitSwitchClosed() {
    return talon1.getSensorCollection().isRevLimitSwitchClosed() == 1;
  }

  public double getEncoderTicks() {
	  return talon1.getSelectedSensorPosition();
  }
  
   // ------------- Process State -------------------

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //TODO: make for when hitting home or limit switch
    // if(1==1) {
    //   isFound = true;
    // }
    
    // if(!isFound) {
    //   setPower(ElevatorConstants.defaultPower);
    // }
    publishTelemetry();
    
  }

  public void publishTelemetry() {
    elevatorPowerEntry.setDouble(talon1.getMotorOutputPercent());
    elevatorPositionEntry.setDouble(getEncoderTicks());
    topLimitSwitchEntry.setBoolean(topLimitSwitchClosed());
    homeLimitSwitchEntry.setBoolean(homeLimitSwitchClosed());
  }
}
