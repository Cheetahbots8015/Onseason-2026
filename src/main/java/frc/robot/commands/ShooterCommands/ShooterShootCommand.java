package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterShootCommand extends Command {

  private final ShooterSubsystem m_subsystem;
  private final double shootVelocity = 0; 
  private final double idleVelocity = 0;

  public ShooterShootCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_subsystem.ShooterVelocityVoltage(shootVelocity);

  }


  @Override
  public void end(boolean interrupted) {
    m_subsystem.ShooterVelocityVoltage(idleVelocity);
  }

  @Override
  public boolean isFinished() {
    return false;

  }

}