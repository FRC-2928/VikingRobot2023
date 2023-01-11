package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.DriverOI;

public class DriverOI {
    private XboxController m_controller;

    public DriverOI(XboxController controller) {
        m_controller = controller;
    }

    // ---------------- Drivetrain ----------------------------

    public Trigger getShiftLowButton() {
        return new JoystickButton(m_controller, XboxController.Button.kX.value);
    }

    public Trigger getShiftHighButton() {
        return new JoystickButton(m_controller, XboxController.Button.kY.value);
    }

    public DoubleSupplier getMoveSupplier() {
        return () -> -m_controller.getLeftY();
    }

    public Trigger getIsAtHighSpeed() {
        return new Trigger(() -> Math.abs(m_controller.getLeftY()) > .85);
    }

    public DoubleSupplier getRotateSupplier() {
        return () -> m_controller.getRightX();
    }  
}