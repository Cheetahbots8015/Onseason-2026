package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ClawShootCommand extends Command {
  private final ClawSubsystem m_subsystem;

  public ClawShootCommand(ClawSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setIntakeVoltage(-1);
    m_subsystem.setShooterVoltage(6);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setIntakeVoltage(0);
    m_subsystem.setShooterVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return m_subsystem.getInput().lightTrigger <= 0.1;
  }
}
