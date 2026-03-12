package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberReverseCommand extends Command {

  private final ClimberSubsystem m_subsystem;

  public ClimberReverseCommand(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setClimberVoltage(-4);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setClimberVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
