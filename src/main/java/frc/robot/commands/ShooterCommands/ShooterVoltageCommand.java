package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterVoltageCommand extends Command {

  private final ShooterSubsystem m_subsystem;

  public ShooterVoltageCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setShooterVoltage(4);
    m_subsystem.setHoodVoltage(4);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setShooterVoltage(0);
    m_subsystem.setHoodVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
