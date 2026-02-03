package frc.robot.commands.BarrelCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.BarrelConstants;
import frc.robot.subsystems.barrel.BarrelSubsystem;

public class BarrelReverseVelocityCommand extends Command {

  private final BarrelSubsystem m_subsystem;

  public BarrelReverseVelocityCommand(BarrelSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setBarrelVelocity(BarrelConstants.BarrelReverseVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
