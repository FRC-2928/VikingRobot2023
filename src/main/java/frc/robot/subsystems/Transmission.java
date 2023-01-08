package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Transmission is responsible for shifting the gear on the drivetrain
 * Contains a statemachine for keeping gear state
 */

public class Transmission extends SubsystemBase {
  private Solenoid m_shiftPiston;
  private Solenoid m_shiftPistonHigh;
  private Solenoid m_shiftPistonLow;
  private GearState m_gearState;

  public enum GearState {
    HIGH, LOW;
  }

  public Transmission() {

    // m_shiftPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticIDs.kDrivetrainShiftSolenoid);
    m_shiftPistonHigh = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticIDs.kDrivetrainShiftSolenoidHigh);
    m_shiftPistonLow = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticIDs.kDrivetrainShiftSolenoidLow);

    m_gearState = GearState.LOW;

  }

  public void setGearState(GearState state) {
    m_gearState = state;

    switch (state) {

    case HIGH:
      setFalse();
      break;

    case LOW:
      setTrue();
      break;
    }
  }

  public void setHigh(){
    setGearState(GearState.HIGH);
  }

  public void setLow(){
    System.out.println("m_transmission::setHigh!!!!");
    setGearState(GearState.LOW);
  }

  public void toggle() {
    setGearState(m_gearState == GearState.LOW ? GearState.HIGH : GearState.LOW);
  }

  public GearState getGearState() {
    return m_gearState;
  }

  private void setTrue() {
    // m_shiftPiston.set(true);
    m_shiftPistonHigh.set(true);
    m_shiftPistonLow.set(false);
  }

  private void setFalse() {
    // m_shiftPiston.set(false);
    m_shiftPistonHigh.set(false);
    m_shiftPistonLow.set(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Gear State", m_gearState.toString());
    // SmartDashboard.putBoolean("Gearstate", m_shiftPiston.isDisabled());
  }
}
