package frc.robot.commands.HoodCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodSubsystem;

public class setHoodVoltageCommand extends Command {

  private final HoodSubsystem m_subsystem;

  public setHoodVoltageCommand(HoodSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setHoodVoltage(SmartDashboard.getNumber("HoodVoltage", 0));
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setHoodVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
