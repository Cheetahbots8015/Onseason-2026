package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeArmSetPositionCommand extends Command {
  private final IntakeSubsystem m_subsystem;
  private double m_position;

  public IntakeArmSetPositionCommand(IntakeSubsystem subsystem, double position) {
    m_subsystem = subsystem;
    m_position = position;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setArmToDegrees(m_position);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getInput().ArmPositionDeg - m_position)
        < IntakeConstants.PositionDeadband;
  }
}
