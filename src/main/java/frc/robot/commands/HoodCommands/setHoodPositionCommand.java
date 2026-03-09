package frc.robot.commands.HoodCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodSubsystem;

public class setHoodPositionCommand extends Command {

  private final HoodSubsystem m_subsystem;

  public setHoodPositionCommand(HoodSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Units: Degress
    m_subsystem.setHoodToDegrees(SmartDashboard.getNumber("HoodPosition", 0));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
