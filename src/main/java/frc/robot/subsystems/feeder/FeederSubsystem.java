// ExampleSubsystem - Subsystem to control a single TalonFX motor for a example

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private boolean isIDLE = true;
  private double targetVelocity = 0.0;

  public FeederSubsystem(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    if (isIDLE) {
      io.idle(inputs);
    } else {
      io.updateOutputs(inputs, targetVelocity);
    }
  }

  public FeederIO getIO() {
    return io;
  }

  public FeederIOInputs getInput() {
    return inputs;
  }

  public void setFeederVoltage(double volts) {
    io.setFeederVoltage(volts);
  }

  public void setTargetVelocity(double velocity) {
    targetVelocity = velocity;
    isIDLE = false;
  }

  public void setIdle() {
    isIDLE = true;
  }
}
