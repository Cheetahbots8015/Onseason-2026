package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ClawShootTimedBackCommand extends Command {
  private final ClawSubsystem m_subsystem;
  private final Timer m_timer = new Timer();

  public ClawShootTimedBackCommand(ClawSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    m_subsystem.setShooterVoltage(-0.5);
    m_subsystem.setIntakeVoltage(0.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setShooterVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > 0.5;
  }
}
