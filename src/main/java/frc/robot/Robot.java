package frc.robot;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Telemetry;

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

		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		this.compressor = new Compressor(1, PneumaticsModuleType.REVPH);
		this.compressor.enableDigital();
		this.robotContainer = new RobotContainer();

		Telemetry.track("Gyroscope", this.robotContainer.drivetrain::readGyro, false);

		try {
			Field field = this.getClass().getSuperclass().getSuperclass().getField("m_watchdog");
			field.setAccessible(true);
			((Watchdog)field.get(this)).setTimeout(1.0 / 10.0); // increase timer to prevent loop overrun messages
		} catch(Exception e) {}
	}

	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		// Get selected routine from the SmartDashboard
		this.autonomousCommand = this.robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if(this.autonomousCommand != null) this.autonomousCommand.schedule();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running which will
		// use the default command which is ArcadeDrive. If you want the autonomous
		// to continue until interrupted by another command, remove
		// this line or comment it out.
		if(this.autonomousCommand != null) this.autonomousCommand.cancel();
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}
}
