package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;

public abstract class OIBase {
	public final XboxController controller;

    protected OIBase(XboxController controller) {
        this.controller = controller;
    }
}
