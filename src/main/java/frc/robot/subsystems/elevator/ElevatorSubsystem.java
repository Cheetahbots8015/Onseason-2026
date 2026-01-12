package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

  private final ElevatorIO io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> setElevatorVoltage(voltage.in(Volt)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void shutdown() {
    io.setElevatorVoltage(0.0);
  }

  public ElevatorIO getIO() {
    return io;
  }

  public void setElevatorVoltage(double volts) {
    io.setElevatorVoltage(volts);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public void VelocityVoltage(double velocity) {
    io.VelocityVoltage(velocity);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public ElevatorIOInputs getInput() {
    io.updateInputs(inputs);
    return inputs;
  }

  public void resetElevatorPosition() {
    io.resetElevetorPosition();
  }
}
