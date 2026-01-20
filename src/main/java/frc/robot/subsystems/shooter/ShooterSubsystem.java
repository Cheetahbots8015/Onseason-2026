package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final SysIdRoutine shooter_sysid =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Shooter/Shooter/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism((voltage) -> setShooterVoltage(voltage.in(Volt)), null, this));

  private final SysIdRoutine hood_sysid =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Shooter/Hood/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism((voltage) -> setHoodVoltage(voltage.in(Volt)), null, this));

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

  public void velocityVoltage(double shooterVelocity, double hoodVelocity) {
    io.velocityVoltage(shooterVelocity, hoodVelocity);
  }

  public void setShooterVoltage(double shooterVolts) {
    io.setShooterVoltage(MathUtil.clamp(shooterVolts, -10, 10));
  }

  public void setHoodVoltage(double hoodVolts) {
    io.setHoodVoltage(MathUtil.clamp(hoodVolts, -10, 10));
  }

  public Command shooterSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return shooter_sysid.quasistatic(direction);
  }

  public Command shooterSysIdDynamic(SysIdRoutine.Direction direction) {
    return shooter_sysid.dynamic(direction);
  }

  public Command hoodSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return hood_sysid.quasistatic(direction);
  }

  public Command hoodSysIdDynamic(SysIdRoutine.Direction direction) {
    return hood_sysid.dynamic(direction);
  }
}
