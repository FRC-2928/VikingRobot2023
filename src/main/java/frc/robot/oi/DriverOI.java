package frc.robot.oi;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Log;

public class DriverOI {
    private XboxController m_controller;

    public DriverOI(XboxController controller) {
        m_controller = controller;
        
        m_controller.a(new EventLoop());
        this.debugAddButtonLog(m_controller::b, "B");
        this.debugAddButtonLog(m_controller::x, "X");
        this.debugAddButtonLog(m_controller::y, "Y");
    }

    private void debugAddButtonLog(Consumer<EventLoop> fn, String name) {
        EventLoop loop = new EventLoop();
        loop.bind(() -> Log.writeln("Button press: " + name));
        fn.accept(loop);
    } 

    // ---------------- Drivetrain ----------------------------

    public DoubleSupplier getRotateSupplier() {
        return () -> m_controller.getRightX();
    }


    public Trigger getShiftLowButton() {
        return new JoystickButton(m_controller, XboxController.Button.kX.value);
    }

    public Trigger getShiftHighButton() {
        return new JoystickButton(m_controller, XboxController.Button.kY.value);
    }


    public Trigger getOrchestraButton(){
        return new JoystickButton(m_controller, XboxController.Button.kStart.value);
    }

    public DoubleSupplier getMoveSupplier() {
        return () -> -m_controller.getLeftY();
    }

    public Trigger getIsAtHighSpeed() {
        return new Trigger(() -> Math.abs(m_controller.getLeftY()) > .85);
    } 

    public Trigger getBalanceButton() {
        return new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    }

    public Trigger getRollButton(){
        return new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    }

    public Trigger getResetGyroButton(){
        return new JoystickButton(m_controller, XboxController.Button.kB.value);
    }
}