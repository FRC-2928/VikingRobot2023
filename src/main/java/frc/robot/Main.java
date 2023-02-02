package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Log;
import frc.robot.subsystems.Telemetry;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what
 * you are doing, do not modify this file except to change the parameter class
 * to the startRobot
 * call.
 */
public final class Main {
	/**
	 * Main initialization function. Do not perform any initialization here.
	 *
	 * <p>
	 * If you change your main robot class, change the parameter type.
	 */
	public static void main(String... args) throws Exception {
		Log.start();
		Log.lineLimit = 50;
		Log.writeln("Robot starting...");

		// publish telemetry to smartdashboard by default
		Telemetry.start(NetworkTableInstance.getDefault().getTable("SmartDashboard"));

		DriverStation.silenceJoystickConnectionWarning(true);

		RobotBase.startRobot(Robot::new);
	}
}
