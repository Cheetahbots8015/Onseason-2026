package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  // Hardware objects
  private final TalonFX shooter;

  private TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
  // Voltage control requests
  final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(0);

  // Inputs from Shooter
  private final StatusSignal<Angle> ShooterPosition;
  private final StatusSignal<AngularVelocity> ShooterVelocity;
  private final StatusSignal<Voltage> ShooterAppliedVolts;
  private final StatusSignal<Current> ShooterCurrent;

  public ShooterIOTalonFX() {
    shooter = new TalonFX(ShooterConstants.shooterID, "rio");
    shooterConfigs.MotorOutput.withNeutralMode(
        ShooterConstants.shooter_neutralmode_Coast
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    shooterConfigs.MotorOutput.withInverted(
        ShooterConstants.shooter_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    shooterConfigs.MotorOutput.withNeutralMode(
        ShooterConstants.shooter_neutralmode_Coast
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    shooterConfigs.MotorOutput.withInverted(
        ShooterConstants.shooter_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // Set PID and feedforward constants from constants file
    shooterConfigs.Slot0.kP = ShooterConstants.shooterkP;
    shooterConfigs.Slot0.kI = ShooterConstants.shooterkI;
    shooterConfigs.Slot0.kD = ShooterConstants.shooterkD;
    shooterConfigs.Slot0.kA = ShooterConstants.shooterkA;
    shooterConfigs.Slot0.kS = ShooterConstants.shooterkS;
    shooterConfigs.Slot0.kV = ShooterConstants.shooterkV;

    // Set current limits
    shooterConfigs.CurrentLimits.StatorCurrentLimit = 40.0;
    shooterConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    // shooterConfigs.CurrentLimits.SupplyCurrentLimit = 40.0;
    // shooterConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply the configuration to the motor
    shooter.getConfigurator().apply(shooterConfigs);

    // Create drive status signals
    ShooterPosition = shooter.getPosition();
    ShooterVelocity = shooter.getVelocity();
    ShooterAppliedVolts = shooter.getMotorVoltage();
    ShooterCurrent = shooter.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ShooterConstants.statusUpdateFrequency,
        ShooterPosition,
        ShooterVelocity,
        ShooterAppliedVolts,
        ShooterCurrent,
        ShooterPosition,
        ShooterVelocity,
        ShooterAppliedVolts,
        ShooterCurrent);
    ParentDevice.optimizeBusUtilizationForAll(shooter);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        ShooterPosition,
        ShooterVelocity,
        ShooterAppliedVolts,
        ShooterCurrent,
        ShooterPosition,
        ShooterVelocity,
        ShooterAppliedVolts,
        ShooterCurrent);
    // Update Shooter inputs
    inputs.ShooterPositionRad = Units.rotationsToRadians(ShooterPosition.getValueAsDouble());
    inputs.ShooterVelocityRotPerSec = ShooterVelocity.getValueAsDouble();
    inputs.ShooterAppliedVolts = ShooterAppliedVolts.getValueAsDouble();
    inputs.ShooterCurrentAmps = ShooterCurrent.getValueAsDouble();
  }

  @Override
  public void ShooterVelocityVoltage(double velocity) {
    shooter.setControl(m_velocity.withVelocity(velocity));
  }

  @Override
  public void setShooterVoltage(double volts) {
    shooter.setVoltage(volts);
  }
}