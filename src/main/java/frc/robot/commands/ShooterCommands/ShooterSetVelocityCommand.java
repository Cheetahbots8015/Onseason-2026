package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterSetVelocityCommand extends Command {

  private final ShooterSubsystem m_subsystem;

  public ShooterSetVelocityCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setTargetVelocity(
        SmartDashboard.getNumber("kShootingSpeed", ShooterConstants.kShootingSpeed));
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setIdle(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
