package frc.robot.subsystems.barrel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.BarrelConstants;

public class BarrelIOTalonFX implements BarrelIO {
  // Hardware objects
  private final TalonFX barrel;
  private TalonFXConfiguration barrelConfigs = new TalonFXConfiguration();

  MotionMagicVelocityVoltage m_velocity = new MotionMagicVelocityVoltage(0).withSlot(0);

  // Inputs from motor
  private final StatusSignal<Angle> BarrelPosition;
  private final StatusSignal<AngularVelocity> BarrelVelocity;
  private final StatusSignal<Voltage> BarrelAppliedVolts;
  private final StatusSignal<Current> BarrelCurrent;

  public BarrelIOTalonFX() {
    barrel = new TalonFX(BarrelConstants.barrelID, "rio");
    barrelConfigs.MotorOutput.withNeutralMode(
        BarrelConstants.barrel_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    // Set motor inversion based on desired rotation direction
    barrelConfigs.MotorOutput.withInverted(
        BarrelConstants.barrel_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // Set PID and feedforward constants from constants file
    barrelConfigs.Slot0.kP = BarrelConstants.barrelkP;
    barrelConfigs.Slot0.kI = BarrelConstants.barrelkI;
    barrelConfigs.Slot0.kD = BarrelConstants.barrelkD;
    barrelConfigs.Slot0.kA = BarrelConstants.barrelkA;
    barrelConfigs.Slot0.kS = BarrelConstants.barrelkS;
    barrelConfigs.Slot0.kV = BarrelConstants.barrelkV;

    barrelConfigs.CurrentLimits.StatorCurrentLimit = BarrelConstants.statorCurrentLimit;
    barrelConfigs.CurrentLimits.StatorCurrentLimitEnable = BarrelConstants.statorCurrentLimitEnable;

    barrelConfigs.CurrentLimits.SupplyCurrentLimit = BarrelConstants.supplyCurrentLimit;
    barrelConfigs.CurrentLimits.SupplyCurrentLimitEnable = BarrelConstants.supplyCurrentLimitEnable;

    // Apply the configuration to the motor
    barrel.getConfigurator().apply(barrelConfigs);

    // Create drive status signals
    BarrelPosition = barrel.getPosition();
    BarrelVelocity = barrel.getVelocity();
    BarrelAppliedVolts = barrel.getMotorVoltage();
    BarrelCurrent = barrel.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        BarrelConstants.statusUpdateFrequency,
        BarrelPosition,
        BarrelVelocity,
        BarrelAppliedVolts,
        BarrelCurrent);
    ParentDevice.optimizeBusUtilizationForAll(barrel);
  }

  @Override
  public void updateInputs(BarrelIOInputs inputs) {
    BaseStatusSignal.refreshAll(BarrelPosition, BarrelVelocity, BarrelAppliedVolts, BarrelCurrent);
    // Update motor inputs
    inputs.BarrelPositionRad = Units.rotationsToDegrees(BarrelPosition.getValueAsDouble());
    inputs.BarrelVelocityRotPerSec = BarrelVelocity.getValueAsDouble();
    inputs.BarrelAppliedVolts = BarrelAppliedVolts.getValueAsDouble();
    inputs.BarrelCurrentAmps = BarrelCurrent.getValueAsDouble();
  }

  @Override
  public void setBarrelVoltage(double volts) {
    barrel.setVoltage(volts);
  }

  @Override
  public void setBarrelVelocity(double velocity) {
    barrel.setControl(m_velocity.withVelocity(velocity));
  }

  @Override
  public void stop() {
    barrel.setVoltage(0);
  }
}
