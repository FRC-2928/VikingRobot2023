// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class FindHome extends CommandBase {

  Elevator m_elevator;
  
  /** Creates a new FindHome. 
   * to be used when elevator is lost (how sad!) which will print on shuffleboard
   * goes up until finding either home or limit switch, and then reset encoders
   * (does not do these things yet)
  */
  public FindHome(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setPower(ElevatorConstants.defaultPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO: make finish when reaching home or limit switch
    return false;
  }
}
