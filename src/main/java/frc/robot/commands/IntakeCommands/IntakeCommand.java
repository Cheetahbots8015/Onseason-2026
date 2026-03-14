package frc.robot.commands.IntakeCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends Command {

  private final IntakeSubsystem m_subsystem;

  public IntakeCommand(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("ArmPosition", 245);
  }

  @Override
  public void execute() {
    if (m_subsystem.getInput().ArmPositionRad > 50) {
      m_subsystem.flywheelVelocityVoltage(SmartDashboard.getNumber("FlyWheelVelocity", 0));
    } else {
      m_subsystem.flywheelStop();
    }
    if (MathUtil.isNear(245, m_subsystem.getInput().ArmPositionRad, 5)) {
      SmartDashboard.putNumber("ArmPosition", 220);
    } else if (MathUtil.isNear(220, m_subsystem.getInput().ArmPositionRad, 5)) {
      SmartDashboard.putNumber("ArmPosition", 245);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.flywheelStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
