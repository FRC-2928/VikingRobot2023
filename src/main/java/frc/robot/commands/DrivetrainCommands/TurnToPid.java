package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class TurnToPid extends ProfiledPIDCommand {
	public TurnToPid(double angle, Drivetrain drivetrain) {
		super(
			new ProfiledPIDController(
				DrivetrainConstants.GainsTurnto.P,
				DrivetrainConstants.GainsTurnto.I,
				DrivetrainConstants.GainsTurnto.D,
				new TrapezoidProfile.Constraints(360, 200)
			),
			() -> drivetrain.readYaw(),
			() -> new TrapezoidProfile.State(angle, 0),
			(output, setpoint) -> drivetrain.tankDriveVolts(output, -output)
		);
		
		this.m_controller.setTolerance(3, 10);
		this.addRequirements(drivetrain);
	}

	@Override
	public boolean isFinished() {
		return this.m_controller.atGoal();
	}
}
