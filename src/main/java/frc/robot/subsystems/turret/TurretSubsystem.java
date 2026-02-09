package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public TurretSubsystem(TurretIO io) {
    this.io = io;
    io.calibrateTurret();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setMotorVoltage(double volts) {
    io.setMotorVoltage(volts);
  }

  /** Set turret position in degrees */
  public void setPosition(double positionDeg) {
    io.setPosition(positionDeg);
  }
}
