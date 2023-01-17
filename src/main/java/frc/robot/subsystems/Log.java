package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Log extends SubsystemBase {
    private static Log instance;
    private StringBuilder log = new StringBuilder(1024);
    private boolean dirty = false;

    private Log() {}

    public static void start() throws Exception {
        Log.instance = new Log();
    }

    public static void write(String str) {
        Log.instance.log.append(str);
        Log.instance.dirty = true;
    }

    public static void writeln(String str) {
        Log.instance.log.append(str);
        Log.instance.log.append('\n');
        Log.instance.dirty = true;
    }

    @Override
    public void periodic() {
        if(this.dirty) {
            SmartDashboard.putString("SystemLog", this.log.toString());
            this.dirty = false;
        }
    }
}
