package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import frc.robot.util.CheetahUtil;

public class ClimberIOTalonFX implements ClimberIO {
  // Hardware objects
  private final TalonFX climber;
  private TalonFXConfiguration climberConfigs = new TalonFXConfiguration();

  final MotionMagicVoltage m_motorRequest = new MotionMagicVoltage(0).withSlot(0);

  // Inputs from motor
  private final StatusSignal<Angle> ClimberPosition;
  private final StatusSignal<AngularVelocity> ClimberVelocity;
  private final StatusSignal<Voltage> ClimberAppliedVolts;
  private final StatusSignal<Current> ClimberCurrent;

  public ClimberIOTalonFX() {
    climber = new TalonFX(ClimberConstants.climberID, "rio");
    climberConfigs.MotorOutput.withNeutralMode(
        ClimberConstants.climber_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    climberConfigs.MotorOutput.withInverted(
        ClimberConstants.climber_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // Set PID and feedforward constants from constants file
    climberConfigs.Slot0.kP = ClimberConstants.climberkP;
    climberConfigs.Slot0.kI = ClimberConstants.climberkI;
    climberConfigs.Slot0.kD = ClimberConstants.climberkD;
    climberConfigs.Slot0.kA = ClimberConstants.climberkA;
    climberConfigs.Slot0.kS = ClimberConstants.climberkS;
    climberConfigs.Slot0.kV = ClimberConstants.climberkV;
    climberConfigs.Slot0.kG = ClimberConstants.climberkG;

    climberConfigs.MotionMagic.MotionMagicCruiseVelocity =
        ClimberConstants.climberMotionMagicCruiseVelocity;
    climberConfigs.MotionMagic.MotionMagicAcceleration = ClimberConstants.climberMotionMagicAcceleration;

    // Apply the configuration to the motor
    climber.getConfigurator().apply(climberConfigs);

    // Create drive status signals
    ClimberPosition = climber.getPosition();
    ClimberVelocity = climber.getVelocity();
    ClimberAppliedVolts = climber.getMotorVoltage();
    ClimberCurrent = climber.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ClimberConstants.statusUpdateFrequency,
        ClimberPosition,
        ClimberVelocity,
        ClimberAppliedVolts,
        ClimberCurrent);
    ParentDevice.optimizeBusUtilizationForAll(climber);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(ClimberPosition, ClimberVelocity, ClimberAppliedVolts, ClimberCurrent);
    // Update motor inputs
    inputs.ClimberPositionDeg = CheetahUtil.climberRotationsToDeg(ClimberPosition.getValueAsDouble());
    inputs.ClimberVelocityRadPerSec = Units.rotationsToRadians(ClimberVelocity.getValueAsDouble());
    inputs.ClimberAppliedVolts = ClimberAppliedVolts.getValueAsDouble();
    inputs.ClimberCurrentAmps = ClimberCurrent.getValueAsDouble();
  }

  @Override
  public void setClimberVoltage(double volts) {
    climber.setVoltage(volts);
  }

  @Override
  public void resetClimberPosition() {
    climber.setPosition(0.0);
  }

  @Override
  public void stopClimber() {
    double currentPos = ClimberPosition.getValueAsDouble();
    climber.setControl(m_motorRequest.withPosition(currentPos));
  }

  @Override
  public void setClimberPosition(double degree) {
    double rotations = CheetahUtil.climberDegToRotations(degree);
    climber.setControl(m_motorRequest.withPosition(rotations));
  }
}
