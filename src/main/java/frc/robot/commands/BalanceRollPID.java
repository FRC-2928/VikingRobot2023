package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class BalanceRollPID extends PIDCommand {
  public static final int ROLL = 1;

  public Drivetrain drive;
  public double time = System.currentTimeMillis();

  public BalanceRollPID(Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(DrivetrainConstants.GainsRollBalance.P, DrivetrainConstants.GainsRollBalance.I,
            DrivetrainConstants.GainsRollBalance.D),
        // This should return the measurement

        () -> {
          double[] angle = new double[3];
          drivetrain.m_pigeon.getYawPitchRoll(angle);

          // double pitch = drivetrain.m_pigeon.getPitch();
          double roll = angle[ROLL];

          SmartDashboard.putNumber("roll", roll);
          return roll;
        },
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          // SmartDashboard.putNumber("Roll",angle[1]);
          SmartDashboard.putNumber("Output", output);
          if (drivetrain.m_pigeon.getRoll() > 0) {
            drivetrain.tankDriveVolts(output, -output);
          } else {
            drivetrain.tankDriveVolts(-output, output);
          }

        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.28, 10);
    SmartDashboard.putData(this.m_controller);
  }

  @Override
  public void initialize() {
    this.time = System.currentTimeMillis();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (System.currentTimeMillis() > time + 7000) {
      return true;
    } else {
      //return getController().atSetpoint();
      return false;
    }
  }
}
