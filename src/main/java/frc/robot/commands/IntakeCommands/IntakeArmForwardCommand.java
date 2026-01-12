package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeArmForwardCommand extends Command {
  private final IntakeSubsystem m_subsystem;
  private double m_volts;

  public IntakeArmForwardCommand(IntakeSubsystem subsystem, double volts) {
    m_subsystem = subsystem;
    m_volts = volts;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_subsystem.getArmPosition() <= 40) {
      m_subsystem.setArmVoltage(m_volts);
    }

    m_subsystem.setIntakeVoltage(6);
    m_subsystem.setIndexerVoltage(2);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setArmVoltage(0.0);
    m_subsystem.setIntakeVoltage(0.0);
    m_subsystem.setIndexerVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
