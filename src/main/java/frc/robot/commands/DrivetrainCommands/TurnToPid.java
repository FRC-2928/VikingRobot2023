package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Log;

public class TurnToPid extends ProfiledPIDCommand {
	private Drivetrain drivetrain;

	public TurnToPid(double angle, Drivetrain drivetrain) {
		super(
			new ProfiledPIDController(
				DrivetrainConstants.GainsTurnto.P,
				DrivetrainConstants.GainsTurnto.I,
				DrivetrainConstants.GainsTurnto.D,
				new TrapezoidProfile.Constraints(360, 200)
			),
			() -> drivetrain.readYaw(),
			new TrapezoidProfile.State(angle, 0),
			(output, setpoint) -> drivetrain.tankDriveVolts(output, -output)
		);
		this.m_controller.setTolerance(3, 10);
		this.addRequirements(drivetrain);
		this.drivetrain = drivetrain;
	}

	@Override
	public void initialize() {
		super.initialize();
		Log.writeln("TurnToPid Start Pose: " + this.drivetrain.getEncoderPose());
	}

	@Override
	public void execute() {
		this.drivetrain.diffDrive.feed();
	}

	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);
		Log.writeln("TurnToPid End Pose: " + this.drivetrain.getEncoderPose());
	}

	@Override
	public boolean isFinished() {
		return this.m_controller.atGoal();
	}
}
