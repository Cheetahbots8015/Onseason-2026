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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  // Hardware objects
  private final TalonFX shooter;
  private final TalonFX hood;

  private TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
  private TalonFXConfiguration hoodConfigs = new TalonFXConfiguration();

  // Voltage control requests
  final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(0);

  // Inputs from Shooter
  private final StatusSignal<Angle> ShooterPosition;
  private final StatusSignal<AngularVelocity> ShooterVelocity;
  private final StatusSignal<Voltage> ShooterAppliedVolts;
  private final StatusSignal<Current> ShooterCurrent;

  private final StatusSignal<Angle> HoodPosition;
  private final StatusSignal<AngularVelocity> HoodVelocity;
  private final StatusSignal<Voltage> HoodAppliedVolts;
  private final StatusSignal<Current> HoodCurrent;

  public ShooterIOTalonFX() {
    shooter = new TalonFX(ShooterConstants.shooterID, "rio");
    hood = new TalonFX(ShooterConstants.hoodID, "rio");

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

    // Second
    hoodConfigs.MotorOutput.withInverted(
        ShooterConstants.hood_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    hoodConfigs.MotorOutput.withNeutralMode(
        ShooterConstants.hood_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    hoodConfigs.MotorOutput.withInverted(
        ShooterConstants.hood_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    hoodConfigs.MotorOutput.withNeutralMode(
        ShooterConstants.hood_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    hoodConfigs.MotorOutput.withInverted(
        ShooterConstants.hood_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // Set PID and feedforward constants from constants file
    shooterConfigs.Slot0.kP = ShooterConstants.shooterkP;
    shooterConfigs.Slot0.kI = ShooterConstants.shooterkI;
    shooterConfigs.Slot0.kD = ShooterConstants.shooterkD;
    shooterConfigs.Slot0.kA = ShooterConstants.shooterkA;
    shooterConfigs.Slot0.kS = ShooterConstants.shooterkS;
    shooterConfigs.Slot0.kV = ShooterConstants.shooterkV;

    // current limit
    shooterConfigs.CurrentLimits.StatorCurrentLimit = 60.0;
    shooterConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    // shooterConfigs.CurrentLimits.SupplyCurrentLimit = 40.0;
    // shooterConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply the configuration to the motor
    shooter.getConfigurator().apply(shooterConfigs);
    hood.getConfigurator().apply(hoodConfigs);

    // Create drive status signals
    ShooterPosition = shooter.getPosition();
    ShooterVelocity = shooter.getVelocity();
    ShooterAppliedVolts = shooter.getMotorVoltage();
    ShooterCurrent = shooter.getTorqueCurrent();

    HoodPosition = shooter.getPosition();
    HoodVelocity = shooter.getVelocity();
    HoodAppliedVolts = shooter.getMotorVoltage();
    HoodCurrent = shooter.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ShooterConstants.statusUpdateFrequency,
        ShooterPosition,
        ShooterVelocity,
        ShooterAppliedVolts,
        ShooterCurrent,
        HoodPosition,
        HoodVelocity,
        HoodAppliedVolts,
        HoodCurrent);
    ParentDevice.optimizeBusUtilizationForAll(shooter);
    ParentDevice.optimizeBusUtilizationForAll(hood);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        ShooterPosition,
        ShooterVelocity,
        ShooterAppliedVolts,
        ShooterCurrent,
        HoodPosition,
        HoodVelocity,
        HoodAppliedVolts,
        HoodCurrent);
    // Update Shooter inputs
    inputs.ShooterPositionRad = Units.rotationsToRadians(ShooterPosition.getValueAsDouble());
    inputs.ShooterVelocityRotPerSec = ShooterVelocity.getValueAsDouble();
    inputs.ShooterAppliedVolts = ShooterAppliedVolts.getValueAsDouble();
    inputs.ShooterCurrentAmps = ShooterCurrent.getValueAsDouble();

    inputs.HoodPositionRad = Units.rotationsToRadians(HoodPosition.getValueAsDouble());
    inputs.HoodVelocityRotPerSec = HoodVelocity.getValueAsDouble();
    inputs.HoodAppliedVolts = HoodAppliedVolts.getValueAsDouble();
    inputs.HoodCurrentAmps = HoodCurrent.getValueAsDouble();
  }

  @Override
  public void ShooterVelocityVoltage(double velocity) {
    shooter.setControl(m_velocity.withVelocity(velocity));
  }
}