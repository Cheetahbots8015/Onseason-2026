package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public ShooterIO getIO() {
    return io;
  }

  public void ShooterVelocityVoltage(double velocity) {
    io.ShooterVelocityVoltage(velocity);
  }

  public void setShooterVoltage(double volts) {
    io.setShooterVoltage(volts);
  }
}