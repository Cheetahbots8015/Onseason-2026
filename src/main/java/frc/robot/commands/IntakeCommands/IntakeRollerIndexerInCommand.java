package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeRollerIndexerInCommand extends Command {
  private final IntakeSubsystem m_subsystem;

  public IntakeRollerIndexerInCommand(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setIntakeVoltage(3);
    m_subsystem.setIndexerVoltage(3);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setIntakeVoltage(0.0);
    m_subsystem.setIndexerVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
