package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ClawAlageShootCommand extends Command {
  private final ClawSubsystem m_subsystem;

  public ClawAlageShootCommand(ClawSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.ClawIntakeTorqueCurrent(-60);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.ClawIntakeTorqueCurrent(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
