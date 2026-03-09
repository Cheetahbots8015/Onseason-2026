package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double targetVelocity = 0.0; // Default target velocity

  private final SysIdRoutine sysId;

  private boolean isIDLE = true;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(0.5).per(Units.Second),
                Units.Volts.of(5.0),
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setMotorVoltage(voltage.in(Units.Volt)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    if (isIDLE) {
      io.idle(inputs);
    } else {
      io.updateOutputs(inputs, targetVelocity);
    }
  }

  public void setTargetVelocity(double rotPerSec) {
    isIDLE = false;
    targetVelocity = rotPerSec;
  }

  public void setIdle(boolean idle) {
    isIDLE = idle;
  }

  public void setNeutralOut() {
    io.stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public void setMotorVoltage(double volts) {
    io.setMotorVoltage(volts);
  }
}
