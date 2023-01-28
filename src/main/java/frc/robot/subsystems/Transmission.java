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
		if(this.gearState == state) return;

		this.gearState = state;

		switch (state) {
			case HIGH:
				// this.shiftPiston.set(false);
				this.shiftPistonHigh.set(false);
				this.shiftPistonLow.set(true);

				Log.writeln("Transmission: HIGH");
				break;

			case LOW:
				// this.shiftPiston.set(true);
				this.shiftPistonHigh.set(true);
				this.shiftPistonLow.set(false);

				Log.writeln("Transmission: LOW");
				break;
		}
	}

	public void setHigh() {
		this.setGearState(GearState.HIGH);
	}

	public void setLow() {
		this.setGearState(GearState.LOW);
	}

	public void toggle() {
		this.setGearState(this.gearState == GearState.LOW ? GearState.HIGH : GearState.LOW);
	}

	public GearState getGearState() {
		return this.gearState;
	}

	@Override
	public void periodic() {
		SmartDashboard.putString("Gear", this.gearState.toString());
	}
}
