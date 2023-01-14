package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class BalancePID extends PIDCommand {
  private Drivetrain drivetrain;
  
  public BalancePID(Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(DrivetrainConstants.GainsBalance.P, DrivetrainConstants.GainsBalance.I, DrivetrainConstants.GainsBalance.D),
        // This should return the measurement
        () -> {
          double[] angle = new double[3];
          drivetrain.m_pigeon.getYawPitchRoll(angle);

          //double pitch = drivetrain.m_pigeon.getPitch();
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

        //maybe omit tolerance? not sure what it does but works well with 10 (tippy with 5)
        //potentially unit of encoder ticks on pigeon
    this.m_controller.setTolerance(10.0);
    this.addRequirements(drivetrain);
    this.drivetrain = drivetrain;


    SmartDashboard.putData(this.m_controller);

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void execute() {
    super.execute();
    
    SmartDashboard.putNumber("I&D values", this.m_controller.getVelocityError());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
