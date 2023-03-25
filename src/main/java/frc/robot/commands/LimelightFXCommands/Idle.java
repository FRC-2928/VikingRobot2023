package frc.robot.commands.LimelightFXCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightFX;
import frc.robot.subsystems.LimelightFX.Color;

public class Idle extends CommandBase {
    private final LimelightFX fx;

    public Idle(LimelightFX fx) {
        this.fx = fx;

        this.addRequirements(fx);
    }

    @Override
    public void initialize() {
        this.fx.addressable();
        this.fx.clear();

        this.fx.circle(5, 5, 4, new Color(255, 127, 255));
    }

    @Override
    public void end(boolean interrupted) {
        this.fx.clear();
    }
}
