package frc.robot.subsystems.turret;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public TurretSubsystem(TurretIO io) {
    this.io = io;
    io.calibrateTurret();
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setMotorVoltage(voltage.in(Units.Volt)), null, this));
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
