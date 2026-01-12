package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorSetPositionCommand extends Command {
  private final ElevatorSubsystem m_subsystem;
  private final double m_position;

  public ElevatorSetPositionCommand(ElevatorSubsystem subsystem, double position) {
    m_subsystem = subsystem;
    m_position = position;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setPosition(m_position);
  }

  @Override
  public void end(boolean interrupted) {
    // if (m_position <= ElevatorConstants.ElevatorResetPosition) {
    //   m_subsystem.resetElevatorPosition();
    // }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getInput().ElevatorHeightMeters - m_position)
        < ElevatorConstants.PositionDeadband;
  }
}
