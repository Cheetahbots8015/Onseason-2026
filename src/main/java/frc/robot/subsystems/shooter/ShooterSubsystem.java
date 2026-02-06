package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void shutdown() {
    io.setMotorVoltage(0.0);
  }

  public void setMotorVoltage(double volts) {
    io.setMotorVoltage(volts);
  }

  public void setVelocityControl(double rotPerSec) {
    io.setVelocityControl(rotPerSec);
  }
}
