package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;

public class pivotElevatorSetPositionCommand extends Command {
  private final ElevatorSubsystem m_elevator;
  private final PivotSubsystem m_pivot;

  private final double elevatorPosition;
  private final double pivotPosition;

  private boolean keepPivot = false;
  private boolean keepElevator = false;

  public pivotElevatorSetPositionCommand(
      ElevatorSubsystem elevator,
      double elevatorPosition,
      PivotSubsystem pivot,
      double pivotPosition) {
    m_elevator = elevator;
    m_pivot = pivot;
    this.elevatorPosition = elevatorPosition;
    this.pivotPosition = pivotPosition;
    addRequirements(m_elevator, m_pivot);
  }

  public pivotElevatorSetPositionCommand(
      ElevatorSubsystem elevator, double elevatorPosition, PivotSubsystem pivot) {
    m_elevator = elevator;
    m_pivot = pivot;
    this.elevatorPosition = elevatorPosition;
    this.pivotPosition = pivot.getInput().PositionRad;
    keepPivot = true;
    addRequirements(m_elevator, m_pivot);
  }

  public pivotElevatorSetPositionCommand(
      ElevatorSubsystem elevator, PivotSubsystem pivot, double pivotPosition) {
    m_elevator = elevator;
    m_pivot = pivot;
    this.elevatorPosition = elevator.getInput().ElevatorHeightMeters;
    keepElevator = true;
    this.pivotPosition = pivotPosition;
    keepPivot = true;
    addRequirements(m_elevator, m_pivot);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!keepElevator) {
      m_elevator.setPosition(elevatorPosition);
    }
    if (!keepPivot) {
      m_pivot.setPosition(pivotPosition);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  private boolean isElevatorAtGoal() {
    return (keepElevator)
        || (Math.abs(m_elevator.getInput().ElevatorHeightMeters - elevatorPosition)
            < ElevatorConstants.PositionDeadband);
  }

  private boolean isPivotAtGoal() {
    return (keepPivot)
        || (Math.abs(m_pivot.getInput().PivotPositionDegree - pivotPosition)
            < PivotConstants.PositionDeadband);
  }

  @Override
  public boolean isFinished() {
    return isElevatorAtGoal() && isPivotAtGoal();
  }
}
