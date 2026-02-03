// ExampleSubsystem - Subsystem to control a single TalonFX motor for a example

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Climber/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setClawVoltage(voltage.in(Volt)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void runVelocity(double clawOutput) {
    io.setOpenLoop(clawOutput);
  }

  public void shutdown() {
    io.setOpenLoop(0.0);
  }

  public ClimberIO getIO() {
    return io;
  }

  public void setClawVoltage(double volts) {
    io.setClawVoltage(volts);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public ClimberIOInputs getInput() {
    return inputs;
  }

  public void setClawPosition(double rotation) {
    io.clawMotionMagic(rotation);
  }

  public void extendClaw() {
    setClawPosition(ClimberConstants.CLAW_EXTEND_POSITION);
  }

  public void retractClaw() {
    setClawPosition(ClimberConstants.CLAW_RETRACT_POSITION);
  }
}
