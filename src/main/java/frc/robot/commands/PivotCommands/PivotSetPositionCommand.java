package frc.robot.commands.PivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.pivot.PivotSubsystem;

public class PivotSetPositionCommand extends Command {
  private final PivotSubsystem m_subsystem;
  private final double m_position;
  private final double m_checkpoint;

  public PivotSetPositionCommand(PivotSubsystem subsystem, double position) {
    m_subsystem = subsystem;
    m_position = position;
    m_checkpoint = Double.NEGATIVE_INFINITY;
    addRequirements(subsystem);
  }

  public PivotSetPositionCommand(PivotSubsystem subsystem, double position, double checkpoint) {
    m_subsystem = subsystem;
    m_position = position;
    m_checkpoint = checkpoint;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setPosition(m_position);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getInput().PivotPositionDegree - m_position)
            < PivotConstants.PositionDeadband
        || Math.abs(m_subsystem.getInput().PivotPositionDegree - m_checkpoint)
            < PivotConstants.PositionDeadband;
  }
}
