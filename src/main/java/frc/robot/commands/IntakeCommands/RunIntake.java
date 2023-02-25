package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
	private Intake intake = new Intake();
	private double speed = 0;

	public RunIntake(Intake intake, double speed) {
		this.intake = intake;
		this.addRequirements(intake);
		this.speed = speed;
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		this.intake.setOutput(this.speed);
	}

	@Override
	public void end(boolean interrupted) {
		this.intake.setOutput(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
