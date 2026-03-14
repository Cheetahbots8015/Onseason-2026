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

  private final SysIdRoutine sysIdleft;
  private final SysIdRoutine sysIdright;

  private boolean isIDLE = true;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;

    sysIdleft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(0.5).per(Units.Second),
                Units.Volts.of(5.0),
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setleftMotorVoltage(voltage.in(Units.Volt)), null, this));

  
    sysIdright =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(0.5).per(Units.Second),
                Units.Volts.of(5.0),
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setrightMotorVoltage(voltage.in(Units.Volt)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setNeutralOut() {
    io.stop();
  }

  public Command leftsysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdleft.quasistatic(direction);
  }

  public Command leftsysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdleft.dynamic(direction);
  }

  public Command rightsysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdright.quasistatic(direction);
  }

  public Command rightsysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdright.dynamic(direction);
  }

  public void setleftMotorVoltage(double volts) {
    io.setleftMotorVoltage(volts);
  }

  public void setrightMotorVoltage(double volts) {
    io.setrightMotorVoltage(volts);
  }

  public double getMotorVelocity() {
    return inputs.rightVelocityRadPerSec;
  }

  public void setLeftVelocityVoltage(double velocity){
    io.se
  }
}
