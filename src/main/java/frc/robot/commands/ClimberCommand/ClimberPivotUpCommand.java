package frc.robot.commands.ClimberCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberPivotUpCommand extends Command {
  private final ClimberSubsystem m_subsystem;

  public ClimberPivotUpCommand(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setPivotVoltage(2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPivotVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Climber Status", m_subsystem.getInput().lightTrigger2 >= 0.9);
    return m_subsystem.getInput().lightTrigger2 >= 0.9;
  }
}
