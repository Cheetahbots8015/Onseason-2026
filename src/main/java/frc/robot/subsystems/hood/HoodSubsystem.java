package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem responsible for controlling the shooter hood mechanism. *
 *
 * <p>This subsystem utilizes a hardware abstraction layer (HoodIO) to support real hardware,
 * physics simulation, and log-based replay.
 */
public class HoodSubsystem extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  /**
   * Constructs a new HoodSubsystem.
   *
   * @param io The IO implementation (Real, Sim, or Replay).
   */
  public HoodSubsystem(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  /**
   * Sets the hood motor to a specific voltage.
   *
   * @param volts Voltage to apply.
   */
  public void setHoodVoltage(double volts) {
    io.setHoodVoltage(volts);
  }

  /**
   * Commands the hood to a specific angle using Motion Magic.
   *
   * @param degrees Target angle in degrees.
   */
  public void setHoodToDegrees(double degrees) {
    io.setHoodToDegrees(degrees);
  }

  /**
   * Returns the current state of the hood subsystem.
   *
   * @return The latest telemetry inputs.
   */
  public HoodIOInputs getInputs() {
    return inputs;
  }

  public boolean isAtSetpoint(double targetDegrees, double tolerance) {
    return Math.abs(inputs.HoodPositionDeg - targetDegrees) < tolerance;
  }
}
