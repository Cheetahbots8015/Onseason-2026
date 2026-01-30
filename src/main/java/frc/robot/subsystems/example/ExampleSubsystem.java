// ExampleSubsystem - Subsystem to control a single TalonFX motor for a example

package frc.robot.subsystems.example;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.example.ExampleIO.ExampleIOInputs;
import org.littletonrobotics.junction.Logger;

public class ExampleSubsystem extends SubsystemBase {
  private final ExampleIO io;
  private final ExampleIOInputsAutoLogged inputs = new ExampleIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public ExampleSubsystem(ExampleIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Example/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> setIntakeVoltage(voltage.in(Volt)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Example", inputs);
  }

  public void runVelocity(double motorOutput) {
    io.setOpenLoop(motorOutput);
  }

  public void shutdown() {
    io.setOpenLoop(0.0);
  }

  public ExampleIO getIO() {
    return io;
  }

  public void setIntakeVoltage(double volts) {
    io.setMotorVoltage(volts);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public ExampleIOInputs getInput() {
    return inputs;
  }

  public void MotorVelocityVoltage(double velocity) {
    io.MotorVelocityVoltage(velocity);
  }

  public void MotorTorqueCurrent(double current) {
    io.MotorTorqueCurrent(current);
  }
}
