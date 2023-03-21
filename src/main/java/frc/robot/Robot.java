package frc.robot;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Log;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static Robot instance;

	public Command autonomousCommand;
	public Compressor compressor;
	public RobotContainer robotContainer;

	@Override
	public void robotInit() {
		Robot.instance = this;

		this.compressor = new Compressor(1, PneumaticsModuleType.REVPH);
		this.compressor.enableDigital();

		this.robotContainer = new RobotContainer();

		try {
			Field field = this.getClass().getSuperclass().getSuperclass().getDeclaredField("m_watchdog");
			field.setAccessible(true);
			((Watchdog)field.get(this)).setTimeout(1.0 / 10.0); // increase timer to prevent loop overrun messages
		} catch(Exception e) {
			Log.error(e);
		}
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		this.robotContainer.drivetrain.setBrakeMode();

		this.autonomousCommand = this.robotContainer.getAutonomousCommand();

		// Start auto commands when starting auto
		if(this.autonomousCommand != null) this.autonomousCommand.schedule();
	}

	@Override
	public void autonomousExit() {
		if(this.autonomousCommand != null) this.autonomousCommand.cancel();
	}

	@Override
	public void teleopInit() {
		this.robotContainer.drivetrain.setCoastMode();
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}
}
