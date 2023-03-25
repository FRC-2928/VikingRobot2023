package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LimelightFX.Behavior;
import frc.robot.subsystems.LimelightFX.Behaviors;

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
	public final Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
	public final RobotContainer robotContainer = new RobotContainer();

	private final Behavior autonomousCountdown = new Behaviors.CountdownBehavior();
	private final Command autonomousCountdownCommand = this.robotContainer.fx.runOnce(() -> this.robotContainer.fx.behavior(this.autonomousCountdown));

	@Override
	public void robotInit() {
		Robot.instance = this;

		this.compressor.enableDigital();

		CommandScheduler.getInstance().setPeriod(this.getPeriod() * 2); // dont let watchdog complain unless we hit twice our loop period
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

		this.autonomousCountdownCommand.schedule();
	}

	@Override
	public void autonomousExit() {
		if(this.autonomousCommand != null) this.autonomousCommand.cancel();
		
		this.autonomousCountdownCommand.cancel();
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
