package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    SmartDashboard.putNumber("Shooter Velocity", 0);
    SmartDashboard.putNumber("Hood Velocity", 0);

    SmartDashboard.putNumber("Shooter Idle", 0);
    SmartDashboard.putNumber("Hood Idle", 0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public ShooterIO getIO() {
    return io;
  }

  public void ShooterVelocityVoltage(double shooterVelocity, double hoodVelocity) {
    io.velocityVoltage(shooterVelocity, hoodVelocity);
  }

  public void ShooterVelocityTorqueCurrentFoc(double shooterVelocity, double hoodVelocity) {
    io.velocityTorqueCurrentFoc(shooterVelocity, hoodVelocity);
  }
}
