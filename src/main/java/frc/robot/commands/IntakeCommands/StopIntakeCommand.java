package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class StopIntakeCommand extends Command {

  private final IntakeSubsystem m_subsystem;

  public StopIntakeCommand(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_subsystem.flywheelStop();
    m_subsystem.ArmPositionVoltage(50);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
