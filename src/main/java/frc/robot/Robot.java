package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	private Command autonomousCommand;
	private Compressor compressor;
	private RobotContainer robotContainer;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		this.compressor = new Compressor(1, PneumaticsModuleType.REVPH);
		Log.writeln(this.compressor.getCurrent());
		this.compressor.enableDigital();
		this.robotContainer = new RobotContainer();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and
	 * test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and
	 * SmartDashboard integrated updating.
	 */
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
		// Set the team alliance
		robotContainer.setAlliance(DriverStation.getAlliance());
		
		// Get selected routine from the SmartDashboard
		this.autonomousCommand = this.robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if(this.autonomousCommand != null) {
			this.autonomousCommand.schedule();
		}
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running which will
		// use the default command which is ArcadeDrive. If you want the autonomous
		// to continue until interrupted by another command, remove
		// this line or comment it out.
		if(this.autonomousCommand != null) {
			this.autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		double[] gyro = this.robotContainer.drivetrain.readGyro();
		SmartDashboard.putNumberArray("gyro", gyro);
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}
}
