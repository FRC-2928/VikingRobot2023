package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
	private final Intake intake;
	private final double speed;

	public RunIntake(Intake intake, double speed) {
		this.intake = intake;
		this.speed = speed;

		this.addRequirements(intake);
	}

	@Override
	public void execute() {
		this.intake.setOutput(this.speed);
	}

	@Override
	public void end(boolean interrupted) {
		this.intake.setOutput(0);
	}
}
