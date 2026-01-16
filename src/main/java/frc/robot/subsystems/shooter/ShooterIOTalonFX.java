package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
  final VelocityTorqueCurrentFOC m_velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0).withSlot(1);

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

    shooterConfigs.Slot1.kP = ShooterConstants.foc_shooterkP;
    shooterConfigs.Slot1.kI = ShooterConstants.foc_shooterkI;
    shooterConfigs.Slot1.kD = ShooterConstants.foc_shooterkD;
    shooterConfigs.Slot1.kA = ShooterConstants.foc_shooterkA;
    shooterConfigs.Slot1.kS = ShooterConstants.foc_shooterkS;
    shooterConfigs.Slot1.kV = ShooterConstants.foc_shooterkV;

    hoodConfigs.Slot0.kP = ShooterConstants.hoodkP;
    hoodConfigs.Slot0.kI = ShooterConstants.hoodkI;
    hoodConfigs.Slot0.kD = ShooterConstants.hoodkD;
    hoodConfigs.Slot0.kA = ShooterConstants.hoodkA;
    hoodConfigs.Slot0.kS = ShooterConstants.hoodkS;
    hoodConfigs.Slot0.kV = ShooterConstants.hoodkV;

    hoodConfigs.Slot1.kP = ShooterConstants.foc_hoodkP;
    hoodConfigs.Slot1.kI = ShooterConstants.foc_hoodkI;
    hoodConfigs.Slot1.kD = ShooterConstants.foc_hoodkD;
    hoodConfigs.Slot1.kA = ShooterConstants.foc_hoodkA;
    hoodConfigs.Slot1.kS = ShooterConstants.foc_hoodkS;
    hoodConfigs.Slot1.kV = ShooterConstants.foc_hoodkV;

    // 怕死
    shooterConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
    shooterConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
    hoodConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
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
  public void velocityVoltage(double shooterVelocity, double hoodVelocity) {
    shooter.setControl(m_velocity.withVelocity(shooterVelocity).withSlot(0));
    hood.setControl(m_velocity.withVelocity(hoodVelocity).withSlot(0));
  }

  @Override
  public void velocityTorqueCurrentFoc(double shooterVelocity, double hoodVelocity) {
    shooter.setControl(m_velocityTorqueCurrentFOC.withVelocity(shooterVelocity).withSlot(1));
    hood.setControl(m_velocityTorqueCurrentFOC.withVelocity(hoodVelocity).withSlot(1));
  }
}
