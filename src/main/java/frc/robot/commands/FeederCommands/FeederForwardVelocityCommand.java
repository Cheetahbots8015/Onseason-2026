package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FeederConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class FeederForwardVelocityCommand extends Command {

  private final FeederSubsystem m_subsystem;

  public FeederForwardVelocityCommand(FeederSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setFeederVelocityVoltage(
        SmartDashboard.getNumber("FeederForwardVelocity", FeederConstants.feederForwardVelocity));
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
