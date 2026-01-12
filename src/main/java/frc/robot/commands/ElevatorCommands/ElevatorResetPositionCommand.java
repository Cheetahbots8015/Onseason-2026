package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorResetPositionCommand extends Command {
  private final ElevatorSubsystem m_subsystem;

  public ElevatorResetPositionCommand(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setElevatorVoltage(-3.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setElevatorVoltage(0.0);
    m_subsystem.resetElevatorPosition();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
