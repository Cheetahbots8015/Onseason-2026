package frc.robot.commands.ClimberCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberSetPositionCommand extends Command {
  private final ClimberSubsystem m_subsystem;
  private final double m_position;

  public ClimberSetPositionCommand(ClimberSubsystem subsystem, double position) {
    m_subsystem = subsystem;
    m_position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_position < -100) {
      m_subsystem.setClimberPivotDefaultPosition(-420);

    } else {
      m_subsystem.setClimberPivotFinalPosition(360);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPivotVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_position - m_subsystem.getInput().PivotPositionDeg)
        < ClimberConstants.allowableError;
  }
}
