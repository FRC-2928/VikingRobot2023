package frc.robot.commands.LimelightFXCommands;

import java.awt.Rectangle;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.LimelightFX;
import frc.robot.subsystems.LimelightFX.GuardRef;

public class SignalGamePiece extends CommandBase {
    private final LimelightFX fx;
    private final GamePiece piece;

    // cube
    // box 5 1 14 9 0x6600ff 1
    // box 5 1 14 9 0xae6bff 0

    // cone
    // box 9 0 10 2 0xffff00 1
    // box 8 3 11 5 0xffff00 1
    // box 7 6 12 8 0xffff00 1
    // box 6 9 13 9 0xffff00 1
    // box 4 10 15 10 0xffff00 1

    public SignalGamePiece(final LimelightFX fx, final GamePiece piece) {
        this.fx = fx;
        this.piece = piece;

        this.addRequirements(fx);
    }

    @Override
    public void initialize() {
        if(LimelightFXConstants.useImageSignals) {
            switch(this.piece) {
                case Cone: {
                    this.fx.image(LimelightFXConstants.imageCone, Optional.empty());

                    break;
                }
            
                case Cube: {
                    this.fx.image(LimelightFXConstants.imageCube2d, Optional.empty());
                    
                    break;
                }
            }
        } else {
            try(GuardRef guard = this.fx.burst()) {
                switch(this.piece) {
                    case Cone: {
                        this.fx.box(new Rectangle(9, 0, 2, 3), LimelightFXConstants.coneColor, true);
                        this.fx.box(new Rectangle(8, 3, 4, 3), LimelightFXConstants.coneColor, true);
                        this.fx.box(new Rectangle(7, 6, 6, 3), LimelightFXConstants.coneColor, true);
                        this.fx.box(new Rectangle(6, 9, 8, 1), LimelightFXConstants.coneColor, true);
                        this.fx.box(new Rectangle(4, 10, 12, 1), LimelightFXConstants.coneColor, true);
                        
                        break;
                    }
                    
                    case Cube: {
                        this.fx.box(new Rectangle(5, 1, 10, 9), LimelightFXConstants.cubeFillColor, true);
                        this.fx.box(new Rectangle(5, 1, 10, 9), LimelightFXConstants.cubeBorderColor, false);

                        break;
                    }
                }
            }
        }
    }
}
