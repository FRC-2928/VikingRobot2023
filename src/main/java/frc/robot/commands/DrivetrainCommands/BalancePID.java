package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class BalancePID extends PIDCommand {
  public Drivetrain drivetrain;

  public BalancePID(Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(DrivetrainConstants.GainsBalance.P, DrivetrainConstants.GainsBalance.I,
            DrivetrainConstants.GainsBalance.D),
        // This should return the measurement
        () -> {
          double[] angle = new double[3];
          drivetrain.m_pigeon.getYawPitchRoll(angle);

          // double pitch = drivetrain.m_pigeon.getPitch();
          double pitch = angle[2];

          SmartDashboard.putNumber("pitch", pitch);
          return pitch;
        },
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          drivetrain.tankDriveVolts(-output, -output);
          SmartDashboard.putNumber("pid output", output);
        });

    this.m_controller.setTolerance(1.0);
    this.m_controller.setSetpoint(0.0);
    this.m_controller.calculate(0.0);
    SmartDashboard.putData(this.m_controller);

    this.addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.m_controller.atSetpoint();
  }
}
