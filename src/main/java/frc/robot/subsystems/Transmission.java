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
	//private Solenoid shiftPiston;
	private Solenoid shiftPistonHigh;
	private Solenoid shiftPistonLow;
	private GearState gearState;

	public enum GearState {
		HIGH,
		LOW;
	}

	public Transmission() {
		// this.shiftPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticIDs.kDrivetrainShiftSolenoid);
		this.shiftPistonHigh = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticIDs.kDrivetrainShiftSolenoidHigh);
		this.shiftPistonLow = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticIDs.kDrivetrainShiftSolenoidLow);

		this.gearState = GearState.LOW;
	}

	public void setGearState(GearState state) {
		this.gearState = state;

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
		System.out.println("this.transmission::setHigh!!!!");
		setGearState(GearState.LOW);
	}

	public void toggle() {
		setGearState(this.gearState == GearState.LOW ? GearState.HIGH : GearState.LOW);
	}

	public GearState getGearState() {
		return this.gearState;
	}

	private void setTrue() {
		// this.shiftPiston.set(true);
		this.shiftPistonHigh.set(true);
		this.shiftPistonLow.set(false);
	}

	private void setFalse() {
		// this.shiftPiston.set(false);
		this.shiftPistonHigh.set(false);
		this.shiftPistonLow.set(true);
	}

	@Override
	public void periodic() {
		SmartDashboard.putString("Gear State", this.gearState.toString());
	}
}
