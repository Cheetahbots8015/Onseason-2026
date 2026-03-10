package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.barrel.BarrelSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StopShootCommand extends Command {

  private final ShooterSubsystem m_subsystem;
  private final BarrelSubsystem m_barrelsubsystem;
  private final FeederSubsystem m_feedersubsystem;

  public StopShootCommand(
      ShooterSubsystem subsystem,
      BarrelSubsystem barrelsubsystem,
      FeederSubsystem feedersubsystem) {
    m_subsystem = subsystem;
    m_barrelsubsystem = barrelsubsystem;
    m_feedersubsystem = feedersubsystem;

    addRequirements(subsystem, m_barrelsubsystem, m_feedersubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setIdle(true);
    m_barrelsubsystem.stop();
    m_feedersubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
