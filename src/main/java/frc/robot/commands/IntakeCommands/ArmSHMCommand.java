package frc.robot.commands.IntakeCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ArmSHMCommand extends Command {

  private final IntakeSubsystem m_subsystem;

  public ArmSHMCommand(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("ArmPosition", 100);
  }

  @Override
  public void execute() {
    if (MathUtil.isNear(100, m_subsystem.getInput().ArmPositionRad, 10)) {
      SmartDashboard.putNumber("ArmPosition", 50);
    } else if (MathUtil.isNear(50, m_subsystem.getInput().ArmPositionRad, 10)) {
      SmartDashboard.putNumber("ArmPosition", 100);
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("ArmPosition", 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
