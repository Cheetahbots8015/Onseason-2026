package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ClimberConstants;

public class ClimberIOTalonFX implements ClimberIO {
  // Hardware objects
  private final TalonFX claw;
  private TalonFXConfiguration clawConfigs = new TalonFXConfiguration();

  final MotionMagicVoltage m_motorRequest = new MotionMagicVoltage(0).withSlot(0);

  // Inputs from motor
  private final StatusSignal<Angle> ClawPosition;
  private final StatusSignal<AngularVelocity> ClawVelocity;
  private final StatusSignal<Voltage> ClawAppliedVolts;
  private final StatusSignal<Current> ClawCurrent;

  public ClimberIOTalonFX() {
    claw = new TalonFX(ClimberConstants.clawID, "canivore");
    clawConfigs.MotorOutput.withNeutralMode(
        ClimberConstants.claw_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    clawConfigs.MotorOutput.withInverted(
        ClimberConstants.claw_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // Set PID and feedforward constants from constants file
    clawConfigs.Slot0.kP = ClimberConstants.clawkP;
    clawConfigs.Slot0.kI = ClimberConstants.clawkI;
    clawConfigs.Slot0.kD = ClimberConstants.clawkD;
    clawConfigs.Slot0.kA = ClimberConstants.clawkA;
    clawConfigs.Slot0.kS = ClimberConstants.clawkS;
    clawConfigs.Slot0.kV = ClimberConstants.clawkV;
    clawConfigs.Slot0.kG = ClimberConstants.clawkG;

    clawConfigs.MotionMagic.MotionMagicCruiseVelocity =
        ClimberConstants.clawMotionMagicCruiseVelocity;
    clawConfigs.MotionMagic.MotionMagicAcceleration = ClimberConstants.clawMotionMagicAcceleration;

    // Apply the configuration to the motor
    claw.getConfigurator().apply(clawConfigs);

    // Create drive status signals
    ClawPosition = claw.getPosition();
    ClawVelocity = claw.getVelocity();
    ClawAppliedVolts = claw.getMotorVoltage();
    ClawCurrent = claw.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ClimberConstants.statusUpdateFrequency,
        ClawPosition,
        ClawVelocity,
        ClawAppliedVolts,
        ClawCurrent);
    ParentDevice.optimizeBusUtilizationForAll(claw);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(ClawPosition, ClawVelocity, ClawAppliedVolts, ClawCurrent);
    // Update motor inputs
    inputs.ClawPositionRad = Units.rotationsToRadians(ClawPosition.getValueAsDouble());
    inputs.ClawVelocityRadPerSec = Units.rotationsToRadians(ClawVelocity.getValueAsDouble());
    inputs.ClawAppliedVolts = ClawAppliedVolts.getValueAsDouble();
    inputs.ClawCurrentAmps = ClawCurrent.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double clawOutput) {
    claw.setControl(new DutyCycleOut(clawOutput));
  }

  @Override
  public void setClawVoltage(double volts) {
    claw.setVoltage(volts);
  }

  @Override
  public void resetClawPosition() {
    claw.setPosition(0.0);
  }

  @Override
  public void stopClaw() {
    double currentPos = ClawPosition.getValueAsDouble();
    claw.setControl(m_motorRequest.withPosition(currentPos));
  }
}
