package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ArmSetMotionMagicCommand extends Command {

  private final IntakeSubsystem m_subsystem;
  private final double m_position;

  public ArmSetMotionMagicCommand(IntakeSubsystem subsystem, double position) {
    m_subsystem = subsystem;
    m_position = position;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.ArmPositionVoltage(m_position);
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
