package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterShootCurrentCommand extends Command {

  private final ShooterSubsystem m_subsystem;
  private final double shootVelocity = 55;
  private final double idleVelocity = 3;

  public ShooterShootCurrentCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.ShooterVelocityTorqueCurrentFoc(shootVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.ShooterVelocityTorqueCurrentFoc(idleVelocity);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
