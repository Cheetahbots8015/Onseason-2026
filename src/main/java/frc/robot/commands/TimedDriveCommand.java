package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class TimedDriveCommand extends Command {
  private final Drive m_subsystem;
  private final Timer m_timer = new Timer();
  private final double m_time;

  public TimedDriveCommand(Drive subsystem, double time) {
    m_subsystem = subsystem;
    m_time = time;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_timer.start();
    m_timer.reset();
  }

  @Override
  public void execute() {
    ChassisSpeeds drivSpeeds = new ChassisSpeeds(-0.5, 0, 0);
    m_subsystem.runVelocity(drivSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds drivSpeeds = new ChassisSpeeds(0, 0, 0);
    m_subsystem.runVelocity(drivSpeeds);
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > m_time;
  }
}
