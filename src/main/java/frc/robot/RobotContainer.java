// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
//import frc.robot.commands.BalanceNative;
import frc.robot.commands.BalancePID;
import frc.robot.commands.Roll_corectionPID;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transmission;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // The Robot's Subsystems
  public final Transmission m_transmission = new Transmission();
  public final Drivetrain m_drivetrain = new Drivetrain(m_transmission::getGearState);

  // XBox Controllers
  private final XboxController m_driverController = new XboxController(0);
  private final DriverOI m_driverOI = new DriverOI(m_driverController);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** 
   * The container for the robot. Contains subsystems, OI devices, and commands. 
   */
  public RobotContainer() {

    // Configure the button bindings
    configureAutoChooser();

    // Configure default commands, button bindings, and shuffleboard
    configureSubsystems();
  }

  /**
   * Configure all subsystems with their default command, button commands,
   * and Shuffleboard output
   */
  private void configureSubsystems() {
    configureDrivetrain();
  }

   /**
   * Configure Drivetrain
   */
  public void configureDrivetrain() {
    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_drivetrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_drivetrain.m_diffDrive.arcadeDrive(m_driverOI.getMoveSupplier().getAsDouble() * 0.4, m_driverOI.getRotateSupplier().getAsDouble() * 0.4),
              m_drivetrain));

    // Configure button commands
    m_driverOI.getShiftLowButton().onTrue(new InstantCommand(m_transmission::setLow, m_transmission));
    m_driverOI.getShiftHighButton().onTrue(new InstantCommand(m_transmission::setHigh, m_transmission));
    m_driverOI.getBalanceButton().whileTrue(new SequentialCommandGroup(new Roll_corectionPID(this.m_drivetrain),new BalancePID(this.m_drivetrain)));
    m_driverOI.getResetGyroButton().onTrue(new InstantCommand(m_drivetrain::zeroGyro, m_drivetrain));

  }

  //Unused balance command
 /*
    private Command Balance() {
    new SequentialCommandGroup(new Roll_corectionPID(this.m_drivetrain),new BalancePID(this.m_drivetrain));
    return null;
  }
  */


  /**
   * Configure AutoChooser 
   */
  private void configureAutoChooser() {
    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
