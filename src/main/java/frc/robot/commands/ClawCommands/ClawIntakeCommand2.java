package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ClawIntakeCommand2 extends Command {
  private final ClawSubsystem claw;

  public ClawIntakeCommand2(ClawSubsystem claw) {
    this.claw = claw;

    addRequirements(claw);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    claw.setIntakeVoltage(3);
    claw.setShooterVoltage(-3);
  }

  @Override
  public void end(boolean interrupted) {
    claw.setIntakeVoltage(0);
    claw.setShooterVoltage(0);
    ;
  }

  @Override
  public boolean isFinished() {
    return claw.getInput().lightTrigger >= 0.9;
  }
}
