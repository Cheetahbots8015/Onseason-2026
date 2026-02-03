package frc.robot.commands.BarrelCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.barrel.BarrelSubsystem;

public class BarrelReverseCommand extends Command {

  private final BarrelSubsystem m_subsystem;

  public BarrelReverseCommand(BarrelSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setBarrelVoltage(SmartDashboard.getNumber("BarrelReverseVoltage", 0));
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setBarrelVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
