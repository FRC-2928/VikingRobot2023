package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

// does not work correctly

public class BalanceNative extends CommandBase {
    private Drivetrain drivetrain;
    public double maxSpeed = 1;

    public BalanceNative(Drivetrain drivetrain) {
        this.addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        this.drivetrain.halt();

        this.drivetrain.setPIDSlot(0);
        this.drivetrain.setPIDPigeonSensors();
        this.drivetrain.setPIDSetpoint(0);
        this.drivetrain.m_diffDrive.setSafetyEnabled(false);
    }

    @Override
    public void execute() {
        System.out.println("a");
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.halt();
        
        this.drivetrain.setPIDSlot(0);
        this.drivetrain.setIntegratedSensors();
        this.drivetrain.m_diffDrive.setSafetyEnabled(true);
        
        this.drivetrain.halt();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
