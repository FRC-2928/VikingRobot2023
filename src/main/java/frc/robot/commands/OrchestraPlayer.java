package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.music.Orchestra;

public class OrchestraPlayer extends CommandBase {
  private Drivetrain drivetrain;
  private final Orchestra player;

  public OrchestraPlayer(Drivetrain drivetrain, String filename) {
    this.drivetrain = drivetrain;

    this.player = new Orchestra();
    this.player.addInstrument(drivetrain.leftLeader);
    this.player.addInstrument(drivetrain.rightLeader);
    this.player.addInstrument(drivetrain.leftFollower);
    this.player.addInstrument(drivetrain.rightFollower);
    this.player.loadMusic(filename);

    this.addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    this.player.play();
  }

  @Override
  public void execute() {
    this.drivetrain.m_diffDrive.feed();
  }

  @Override
  public void end(boolean interrupted) {
    this.player.stop();
  }

  @Override
  public boolean isFinished() {
    return !this.player.isPlaying();
  }
}
