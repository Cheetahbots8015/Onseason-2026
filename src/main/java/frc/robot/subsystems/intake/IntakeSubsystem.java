// RollerSubsystem - Subsystem to control a single TalonFX motor for a roller

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setArmVoltage(voltage.in(Volt)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    SmartDashboard.putBoolean("Canrange Value", io.getCanRange());
  }

  // Stop the indexer motor by setting it to neutral
  public void shutDown() {
    io.setOpenLoop(0.0, 0.0, 0.0);
  }

  public void runVelocity(double indexerOutput, double intakeOutput, double armOutput) {
    io.setOpenLoop(indexerOutput, intakeOutput, armOutput);
  }

  public void setArmVoltage(double volts) {
    io.setArmVoltage(volts);
  }

  public void setIntakeVoltage(double volts) {
    io.setIntakeVoltage(volts);
  }

  public void setIndexerVoltage(double volts) {
    io.setIndexerVoltage(volts);
  }

  public void setArmToDegrees(double degrees) {
    io.setArmToDegrees(degrees);
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

  public void armVelocityVoltage(double velocity) {
    io.armVelocityVoltage(velocity);
  }

  public double getArmPosition() {
    return inputs.ArmPositionDeg;
  }

  public void resetArmPosition() {
    io.resetArmPosition();
  }

  public boolean getCanRange() {
    return io.getCanRange();
  }
}
