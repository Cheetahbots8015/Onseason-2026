package frc.robot.commands.TurretCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class reserTurretCommand extends Command {

  private final TurretSubsystem m_subsystem;

  public reserTurretCommand(TurretSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setPosition(0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return MathUtil.isNear(0, m_subsystem.getPosition(), 2);
  }
}
