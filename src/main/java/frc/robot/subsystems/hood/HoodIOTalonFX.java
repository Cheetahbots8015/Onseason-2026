package frc.robot.subsystems.hood;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.HoodConstants;
import frc.robot.util.CheetahUtil;

public class HoodIOTalonFX implements HoodIO {
  // Hardware objects
  private final TalonFX hood;
  private TalonFXConfiguration hoodConfigs = new TalonFXConfiguration();

  // Torque current control requests
  final PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0);
  // Inputs from motor
  private final StatusSignal<Angle> HoodPosition;
  private final StatusSignal<AngularVelocity> HoodVelocity;
  private final StatusSignal<Voltage> HoodAppliedVolts;
  private final StatusSignal<Current> HoodCurrent;

  public HoodIOTalonFX() {
    hood = new TalonFX(HoodConstants.hoodID, "canivore");
    hoodConfigs.MotorOutput.withNeutralMode(
        HoodConstants.hood_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    hoodConfigs.MotorOutput.withInverted(
        HoodConstants.hood_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // Set PID and feedforward constants from constants file
    hoodConfigs.Slot0.kP = HoodConstants.hoodkP;
    hoodConfigs.Slot0.kI = HoodConstants.hoodkI;
    hoodConfigs.Slot0.kD = HoodConstants.hoodkD;
    hoodConfigs.Slot0.kA = HoodConstants.hoodkA;
    hoodConfigs.Slot0.kS = HoodConstants.hoodkS;
    hoodConfigs.Slot0.kV = HoodConstants.hoodkV;

    hoodConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        HoodConstants.reverseSoftLimitThreshold;
    hoodConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        HoodConstants.forwardSoftLimitThreshold;
    hoodConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = HoodConstants.reverseSoftLimitEnable;
    hoodConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = HoodConstants.forwardSoftLimitEnable;

    // Apply the configuration to the motor
    hood.getConfigurator().apply(hoodConfigs);

    // clear offset
    tryUntilOk(5, () -> hood.setPosition(0.0));

    // Create drive status signals
    HoodPosition = hood.getPosition();
    HoodVelocity = hood.getVelocity();
    HoodAppliedVolts = hood.getMotorVoltage();
    HoodCurrent = hood.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        HoodConstants.statusUpdateFrequency,
        HoodPosition,
        HoodVelocity,
        HoodAppliedVolts,
        HoodCurrent);
    ParentDevice.optimizeBusUtilizationForAll(hood);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(HoodPosition, HoodVelocity, HoodAppliedVolts, HoodCurrent);
    // Update motor inputs
    inputs.HoodPositionDeg = CheetahUtil.hoodRotationToDegrees(HoodPosition.getValueAsDouble());
    inputs.HoodVelocityRadPerSec = Units.rotationsToRadians(HoodVelocity.getValueAsDouble());
    inputs.HoodAppliedVolts = HoodAppliedVolts.getValueAsDouble();
    inputs.HoodCurrentAmps = HoodCurrent.getValueAsDouble();
  }

  @Override
  public void setHoodVoltage(double volts) {
    hood.setVoltage(volts);
  }

  @Override
  public void setHoodToDegrees(double degrees) {
    double rotation = CheetahUtil.hoodDegreesToRotation(degrees);
    hood.setControl(m_PositionVoltage.withPosition(rotation));
  }
}
