package frc.robot.commands.IntakeCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class TimedIntakeCommand extends Command {

  private final IntakeSubsystem m_subsystem;
  private Timer m_timer;
  private double m_time;

  public TimedIntakeCommand(IntakeSubsystem subsystem, double time) {
    m_subsystem = subsystem;
    m_time = time;
    m_timer = new Timer();
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    SmartDashboard.putNumber("ArmPosition", 240);
  }

  @Override
  public void execute() {
    if (m_subsystem.getInput().ArmPositionRad > 50) {
      m_subsystem.flywheelVelocityVoltage(SmartDashboard.getNumber("FlyWheelVelocity", 0));
    } else {
      m_subsystem.flywheelStop();
    }
    if (MathUtil.isNear(240, m_subsystem.getInput().ArmPositionRad, 5)) {
      SmartDashboard.putNumber("ArmPosition", 230);
    } else if (MathUtil.isNear(230, m_subsystem.getInput().ArmPositionRad, 5)) {
      SmartDashboard.putNumber("ArmPosition", 240);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.flywheelStop();
    SmartDashboard.putNumber("ArmPosition", 0);
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > m_time;
  }
}
