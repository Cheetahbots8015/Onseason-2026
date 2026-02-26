package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double targetVelocity = 0.0; // Default target velocity
  private boolean idle = true;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    if (idle) {
      io.idle(inputs);
    } else {
      io.updateOutputs(inputs, targetVelocity);
    }
  }

  public void setTargetVelocity(double rotPerSec) {
    this.idle = false;
    targetVelocity = rotPerSec;
  }

  public void setIdle(boolean idle) {
    this.idle = idle;
  }
}
