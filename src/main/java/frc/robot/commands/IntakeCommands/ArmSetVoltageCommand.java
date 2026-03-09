package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ArmSetVoltageCommand extends Command {

  private final IntakeSubsystem m_subsystem;
  private final double m_volts;

  public ArmSetVoltageCommand(IntakeSubsystem subsystem, double volts) {
    m_subsystem = subsystem;
    m_volts = volts;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setArmVoltage(m_volts);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setArmVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
