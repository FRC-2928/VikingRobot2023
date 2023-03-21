package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Telemetry extends SubsystemBase {
	public static class TelemetryData {
		protected String name;
		protected Supplier<Object> supplier;
		protected boolean lazy;

		protected TelemetryData(String name, Supplier<Object> supplier, boolean lazy) {
			this.name = name;
			this.supplier = supplier;
			this.lazy = lazy;
		}

		public void publish() {
			Telemetry.instance.table.getEntry(this.name).setValue(this.supplier.get());
		}

		public void publish(Object value) {
			Telemetry.instance.table.getEntry(this.name).setValue(value);
		}
	}

	private static Telemetry instance;

	private NetworkTable table;
	private Map<String, TelemetryData> telemetry = new HashMap<>();

	private Telemetry(NetworkTable table) {
		this.table = table;
	}

	/// Starts the log, you should never need to use this, save for
	/// the first line of the `main` function
	public static void start(NetworkTable table) {
		Telemetry.instance = new Telemetry(table);
	}

	public static TelemetryData track(String name, Supplier<Object> supplier, boolean lazy) {
		TelemetryData telem = new TelemetryData(name, supplier, lazy);
		Telemetry.instance.telemetry.put(name, telem);

		if(!lazy) telem.publish();

		return telem;
	}

	@Override
	public void periodic() {
		for(TelemetryData data : this.telemetry.values()) {
			if(!data.lazy) data.publish();
		}
	}
}
