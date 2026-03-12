package frc.robot.commands.TurretCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.TurretSubsystem;

public class reserTurretCommand extends Command {

  private final TurretSubsystem m_subsystem;
  private final Drive m_drive;

  public reserTurretCommand(TurretSubsystem subsystem, Drive drive) {
    m_subsystem = subsystem;
    m_drive = drive;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setPosition(0);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setTurretPigeonOffset();
  }

  @Override
  public boolean isFinished() {
    return MathUtil.isNear(0, m_subsystem.getPosition(), 0.5);
  }
}
