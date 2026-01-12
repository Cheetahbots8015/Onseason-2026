package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeRollerIndexerOutCommand extends Command {
  private final IntakeSubsystem m_subsystem;

  public IntakeRollerIndexerOutCommand(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Set both intake roller and indexer voltages
    if (m_subsystem.getArmPosition() <= 40) {
      m_subsystem.setArmVoltage(1);
    }
    m_subsystem.setIntakeVoltage(-3);
    m_subsystem.setIndexerVoltage(-3);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop both intake roller and indexer
    m_subsystem.setIntakeVoltage(0.0);
    m_subsystem.setIndexerVoltage(0.0);
    m_subsystem.setArmVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
