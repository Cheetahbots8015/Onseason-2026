package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterShootCurrentCommand extends Command {

  private final ShooterSubsystem m_subsystem;

  public ShooterShootCurrentCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.ShooterVelocityTorqueCurrentFoc(
      SmartDashboard.getNumber("Shooter Velocity", 0),
      SmartDashboard.getNumber("Hood Velocity", 0)
    );
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.ShooterVelocityTorqueCurrentFoc(
        SmartDashboard.getNumber("Shooter Idle", 0), SmartDashboard.getNumber("Hood Idle", 0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
