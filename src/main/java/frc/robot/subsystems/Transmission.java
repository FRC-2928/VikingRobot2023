package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transmission extends SubsystemBase {
	private Solenoid shiftPiston;
	private GearState gearState;

	public enum GearState {
		HIGH,
		LOW;
	}

	public Transmission() {
		this.shiftPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticIDs.drivetrainShiftPiston);

		this.gearState = GearState.LOW;

		Telemetry.track("Gear", () -> this.gearState.toString(), false);
	}

	public void shift(GearState state) {
		if(this.gearState == state) return;

		this.gearState = state;

		switch (state) {
			case HIGH:
				this.shiftPiston.set(true);

				Log.writeln("[High Gear]");
				break;

			case LOW:
				this.shiftPiston.set(false);

				Log.writeln("[Low Gear]");
				break;
		}
	}

	public void setHigh() {
		this.shift(GearState.HIGH);
	}

	public void setLow() {
		this.shift(GearState.LOW);
	}

	public void toggle() {
		this.shift(this.gearState == GearState.LOW ? GearState.HIGH : GearState.LOW);
	}

	public GearState getGearState() {
		return this.gearState;
	}
}
