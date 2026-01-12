package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ClawIntakeCommand extends Command {
  private final ClawSubsystem claw;
  private final IntakeSubsystem intake;

  public ClawIntakeCommand(ClawSubsystem claw, IntakeSubsystem intake) {
    this.claw = claw;
    this.intake = intake;
    addRequirements(claw, intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    claw.setIntakeVoltage(4);
    intake.setIndexerVoltage(3);
  }

  @Override
  public void end(boolean interrupted) {
    claw.setIntakeVoltage(0.5);
    intake.setIndexerVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return claw.getInput().lightTrigger >= 0.9;
  }
}
