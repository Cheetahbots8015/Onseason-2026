package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterSetVoltageCommand extends Command {

  private final ShooterSubsystem m_subsystem;

  public ShooterSetVoltageCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setMotorVoltage(
        SmartDashboard.getNumber("kShootingVoltage", ShooterConstants.kShootingVoltage));
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.shutdown();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
