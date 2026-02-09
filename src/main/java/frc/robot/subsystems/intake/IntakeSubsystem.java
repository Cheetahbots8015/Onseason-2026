// ExampleSubsystem - Subsystem to control a single TalonFX motor for a example

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SysIdRoutine sysId;
  private static final double ARM_DEPLOY_POSITION = 1.2;
  private static final double ARM_RETRACT_POSITION = 0.0;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> setFlywheelVoltage(voltage.in(Volt)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void runVelocity(double flywheelOutput, double armOutput) {
    io.setOpenLoop(flywheelOutput, armOutput);
  }

  public void setArmVoltage(double volts) {
    io.setArmVoltage(volts);
  }

  public void setFlywheelVoltage(double volts) {
    io.setFlywheelVoltage(volts);
  }

  public void shutdown() {
    io.setOpenLoop(0.0, 0.0);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public IntakeIOInputs getInput() {
    return inputs;
  }

  public void armMotionMagic(double targetPosition) {
    io.armMotionMagic(targetPosition);
  }

  public void flywheelVelocityVoltage(double targetVelocity) {
    io.flywheelVelocityVoltage(targetVelocity);
  }

  public void flywheelStop() {
    io.flywheelStop();
  }

  public void deployIntake() {
    io.armMotionMagic(ARM_DEPLOY_POSITION);
    io.flywheelVelocityVoltage(0.0);
  }

  public void retractIntake() {
    io.armMotionMagic(ARM_RETRACT_POSITION);
    io.flywheelStop();
  }
}
