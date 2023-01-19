package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Log extends SubsystemBase {
    private static Log instance;

    /// The maximum number of lines the buffer should contain at any given moment
    /// Note that `writelnFast` will bypass this, so ensure you call `trimExcessLeadingLines` after using `writelnFast`
    public static int lineLimit = Integer.MAX_VALUE;
    private int lines = 0;
    private StringBuilder log = new StringBuilder(1024);
    private boolean dirty = false;

    private Log() {}

    /// Starts the log, you should never need to use this, save for the first line of the `main` function
    public static void start() throws Exception {
        Log.instance = new Log();
    }

    /// Writes data to the buffer and dirties the network entry.
    public static void write(String str) {
        Log.instance.log.append(str);
        Log.instance.lines += str.lines().count();
        Log.instance.dirty = true;
    }

    /// Writes a line to the buffer, trims old leading lines, and dirties the network entry.
    public static void writeln(String str) {
        Log.write(str + '\n');
        Log.trimExcessLeadingLines();
    }

    /// Writes a line to the buffer, does NOT trim any old leading lines, and dirties the network entry.
    /// Use this to buffer many line calls together before calling `trimExcessLeadingLines`
    public static void writelnFast(String str) { Log.write(str + '\n'); }

    /// Trims as many leading lines is necessary so that the buffer contains less than `lineLimit` lines long
    public static void trimExcessLeadingLines() {
        while(Log.instance.lines > Log.lineLimit) Log.instance.trimSingleLine();
    }

    /// Unconditionally trims a single line off the front
    private void trimSingleLine() {
        this.log.delete(0, Math.max(this.log.indexOf("\n"), 0));
        this.lines--;
        this.dirty = true;
    }

    @Override
    public void periodic() {
        if(this.dirty) {
            SmartDashboard.putString("SystemLog", this.log.toString());
            this.dirty = false;
        }
    }
}
