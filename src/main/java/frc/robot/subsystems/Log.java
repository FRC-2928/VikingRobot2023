package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Log extends SubsystemBase {
	private static Log instance;

	/// The maximum number of lines the buffer should contain at any given moment
	/// Note that `writelnFast` will bypass this, so ensure you call
	/// `trimExcessLeadingLines` after using `writelnFast`
	public static int lineLimit = Integer.MAX_VALUE;
	private int lines = 0;
	private StringBuilder log = new StringBuilder(1024);
	private boolean dirty = false;

	private Log() {}

	/// Starts the log, you should never need to use this, save for the first line
	/// of the `main` function
	public static void start() throws Exception {
		Log.instance = new Log();
	}

	/// Writes data to the buffer and dirties the network entry.
	public static void write(String str) {
		Log.instance.log.append(str);
		Log.instance.lines += str.lines().count();
		Log.instance.dirty = true;
	}

	/// Writes a line to the buffer, trims old leading lines, and dirties the
	/// network entry.
	public static void writeln(String str) {
		Log.write(str + '\n');
		Log.trimExcessLeadingLines();
	}

	/// Writes a line to the buffer, does NOT trim any old leading lines, and
	/// dirties the network entry.
	/// Use this to buffer many line calls together before calling
	/// `trimExcessLeadingLines` or `writeln`
	public static void writelnFast(String str) { Log.write(str + '\n'); }

	/// Writes an error to the buffer, trims old leading lines, dirties the network entry, and transmits the error to the driver station
	public static void error(Exception error) {
		DriverStation.reportError(error.getLocalizedMessage(), error.getStackTrace());
		Log.writelnFast("❗ ERROR ❗");
		Log.writelnFast(error.getLocalizedMessage());
		Log.writelnFast("❗ More information has been printed in the Driver Station ❗");
	}

	/// Writes a warning to the buffer, trims old leading lines, dirties the network entry, and transmits the error to the driver station
	public static void warning(String warning) {
		DriverStation.reportError(warning, Thread.currentThread().getStackTrace());
		Log.writelnFast("⚠️ WARNING ⚠️");
		Log.writeln(warning);
	}

	/// Trims as many leading lines is necessary so that the buffer contains less
	/// than `lineLimit` lines long
	public static void trimExcessLeadingLines() {
		while (Log.instance.lines > Log.lineLimit) Log.instance.trimSingleLine();
	}

	/// Unconditionally trims a single line off the front
	private void trimSingleLine() {
		System.out.println(this.log.indexOf("\n"));

		this.log.delete(0, Math.max(this.log.indexOf("\n") + 1, 0)); // this doesnt work
		this.lines--;
		this.dirty = true;
	}

	@Override
	public void periodic() {
		if (this.dirty) {
			SmartDashboard.putString("SystemLog", this.log.toString());
			this.dirty = false;
		}
	}
}
