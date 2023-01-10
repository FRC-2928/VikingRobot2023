package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {
    private Drivetrain drivetrain;
    public double maxSpeed = 1;
    
    public Balance(Drivetrain drivetrain, double p, double i, double d) {
        this.addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }
    
    @Override
    public void initialize() {
        drivetrain.halt();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.halt();
    }
}
