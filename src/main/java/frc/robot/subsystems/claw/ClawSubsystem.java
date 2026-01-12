// ClawSubsystem - Subsystem to control a single TalonFX motor for a claw

package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.claw.ClawIO.ClawIOInputs;
import org.littletonrobotics.junction.Logger;

public class ClawSubsystem extends SubsystemBase {
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public ClawSubsystem(ClawIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Claw/Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> setIntakeVoltage(voltage.in(Volt)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);
    SmartDashboard.putBoolean("Claw LightTrigger Status", inputs.lightTrigger >= 0.9);
    SmartDashboard.putNumber("Claw LightTrigger Status", inputs.lightTrigger);
  }

  public void runVelocity(double intakeOutput, double shooterOutput) {
    io.setOpenLoop(intakeOutput, shooterOutput);
  }

  public void shutdown() {
    io.setOpenLoop(0.0, 0.0);
  }

  public ClawIO getIO() {
    return io;
  }

  public void setIntakeVoltage(double volts) {
    io.setIntakeVoltage(volts);
  }

  public void setShooterVoltage(double volts) {
    io.setShooterVoltage(volts);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public ClawIOInputs getInput() {
    return inputs;
  }

  public void IntakeVelocityVoltage(double velocity) {
    io.IntakeVelocityVoltage(velocity);
  }

  public void ClawIntakeTorqueCurrent(double current) {
    io.ClawIntakeTorqueCurrent(current);
  }
}
