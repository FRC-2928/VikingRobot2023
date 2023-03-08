package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.oi.DriverOI;

public class DriverOI extends OIBase {
	/*

	A - 
	B - 
	X - 
	Y - balance

	Start(right) - 
	Back(left) - c-stop

	LB - start pov selector for tag
	RB - runs the intake

	LT - speed reduct
	RT - hold to shift high

	LS - y-move
	RS - x-rotate

	LS Click - shift high
	RS Click - shift low

	*/
	
	public DriverOI(XboxController controller) {
		super(controller);
	}

	// ---------------- Drivetrain ----------------------------

	public DoubleSupplier getMoveSupplier() {
		return () -> -this.controller.getLeftY();
	}

	public DoubleSupplier getRotateSupplier() {
		return () -> this.controller.getRightX();
	}

	// public Trigger getMoveToPlaceHigh() {
	// 	return new JoystickButton(this.controller, XboxController.Button.kB.value);
	// }

	// public Trigger getMoveToPlaceMid() {
	// 	return new JoystickButton(this.controller, XboxController.Button.kA.value);
	// }

	// Shifting

	public Trigger getShiftLowButton() {
		return new JoystickButton(this.controller, XboxController.Button.kRightStick.value);
	}

	public Trigger getShiftHighButton() {
		return new JoystickButton(this.controller, XboxController.Button.kLeftStick.value);
	}

	public Trigger getShiftButton() {
		return new Trigger(() -> this.controller.getRightTriggerAxis() > 0.5);
	}

	/// Get the speed factor to use by taking the reduct trigger (left) into account
	/// The reduction factor can be modified in Constants.DrivetrainConstants.ReductFactor
	public double getReductFactor() {
		return MathUtil.interpolate(1, Constants.DrivetrainConstants.reductFactor, this.controller.getLeftTriggerAxis()); // Pulling the trigger more moves slower
	}

	public double getReductFactorRotation() {
		return MathUtil.interpolate(1, Constants.DrivetrainConstants.reductFactor, this.controller.getLeftTriggerAxis()); // Pulling the trigger more moves slower
	}

	// public Trigger getCoastBrakeButton() {
	// 	return new JoystickButton(this.controller, XboxController.Button.kX.value);
	// }

	// Balance

	public Trigger getBalanceButton() {
		return new JoystickButton(this.controller, XboxController.Button.kY.value);
	}

	// Misc

	public Trigger getApproachTagButton() {
		return new JoystickButton(this.controller, XboxController.Button.kLeftBumper.value);
	}

	public Trigger getRunIntakeButton() {
		return new JoystickButton(this.controller, XboxController.Button.kRightBumper.value);
	}

	// public Trigger getElevatorToStartButton(){
	// 	return new JoystickButton(this.controller, XboxController.Button.kA.value);
	// }
}