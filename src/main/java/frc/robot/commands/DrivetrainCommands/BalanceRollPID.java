// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DrivetrainCommands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Log;

public class BalanceRollPID extends PIDCommand {
  public static final int ROLL = 1;

  public double time = System.currentTimeMillis();

  public BalanceRollPID(Drivetrain drivetrain) {
    super(
      new PIDController(
        DrivetrainConstants.GainsRollBalance.P,
        DrivetrainConstants.GainsRollBalance.I,
        DrivetrainConstants.GainsRollBalance.D
      ),
      () -> {
        double roll = drivetrain.readGyro()[1];

        SmartDashboard.putNumber("roll", roll);
        return roll;
      },
      0,
      output -> {
        if (drivetrain.m_pigeon.getRoll() > 0) {
          drivetrain.tankDriveVolts(output, -output);
        } else {
          drivetrain.tankDriveVolts(-output, output);
        }
      }
    );
    
    this.m_controller.setTolerance(0.3);
    this.m_controller.setSetpoint(0.0);
    this.m_controller.calculate(0.0);
    SmartDashboard.putData(this.m_controller);

    this.addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    this.time = System.currentTimeMillis();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Log.writeln("error: " + this.m_controller.getPositionError() + ", " + this.m_controller.atSetpoint());
    return System.currentTimeMillis() > this.time + 7000 || this.m_controller.atSetpoint();
  }
}
