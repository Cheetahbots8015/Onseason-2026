// ExampleSubsystem - Subsystem to control a single TalonFX motor for a example

package frc.robot.subsystems.barrel;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.barrel.BarrelIO.BarrelIOInputs;
import org.littletonrobotics.junction.Logger;

public class BarrelSubsystem extends SubsystemBase {
  private final BarrelIO io;
  private final BarrelIOInputsAutoLogged inputs = new BarrelIOInputsAutoLogged();
  private double targetVelocity = 0.0; // Default target velocity

  private final SysIdRoutine sysId;
  // Default to IDLE state
  private boolean isIDLE = false;

  public BarrelSubsystem(BarrelIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setMotorVoltage(voltage.in(Units.Volt)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Barrel", inputs);

    if (isIDLE) {
      io.idle(inputs);
    } else {
      io.updateOutputs(inputs, targetVelocity);
    }
  }

  public BarrelIO getIO() {
    return io;
  }

  public BarrelIOInputs getInput() {
    return inputs;
  }

  public void setBarrelVoltage(double volts) {
    io.setBarrelVoltage(volts);
  }

  public void setBarrelVelocity(double velocity) {
    isIDLE = false;
    targetVelocity = velocity;
  }

  public void stop() {
    io.stop();
  }

  public void setIdle(boolean idle) {
    isIDLE = idle;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
