package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class FlyWheelSetVelocityCommand extends Command {

  private final IntakeSubsystem m_subsystem;

  public FlyWheelSetVelocityCommand(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setFlyWheelTargetVelocity(SmartDashboard.getNumber("IntakeFlyWheelVelocity", 400));
    ;
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.flyWheelStop();;
    ;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
