package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ShooterConstants;

/** TalonFX-based implementation of ShooterIO for two motors. */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX left;
  private final TalonFX right;

  private TalonFXConfiguration leftConfigs = new TalonFXConfiguration();
  private TalonFXConfiguration rightConfigs = new TalonFXConfiguration();

  // Controls
  private final VelocityVoltage m_velocityLeft = new VelocityVoltage(0).withSlot(0);
  private final VelocityVoltage m_velocityRight = new VelocityVoltage(0).withSlot(0);

  // Status signals
  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftCurrent;

  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightCurrent;

  public ShooterIOTalonFX() {
    left = new TalonFX(ShooterConstants.kLeftMotorID, "canivore");
    right = new TalonFX(ShooterConstants.kRightMotorID, "canivore");

    leftConfigs.MotorOutput.withNeutralMode(
        ShooterConstants.kMotorNeutralCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    rightConfigs.MotorOutput.withNeutralMode(
        ShooterConstants.kMotorNeutralCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    leftConfigs.MotorOutput.withInverted(
        ShooterConstants.kMotorInvertLeftCCWPositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    rightConfigs.MotorOutput.withInverted(
        ShooterConstants.kMotorInvertRightCCWPositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    rightConfigs.Slot0.kP = ShooterConstants.kRightSlot_kP;
    rightConfigs.Slot0.kI = ShooterConstants.kRightSlot_kI;
    rightConfigs.Slot0.kD = ShooterConstants.kRightSlot_kD;
    rightConfigs.Slot0.kA = ShooterConstants.kRightSlot_kA;
    rightConfigs.Slot0.kS = ShooterConstants.kRightSlot_kS;
    rightConfigs.Slot0.kV = ShooterConstants.kRightSlot_kV;

    leftConfigs.Slot0.kP = ShooterConstants.kLeftSlot_kP;
    leftConfigs.Slot0.kI = ShooterConstants.kLeftSlot_kI;
    leftConfigs.Slot0.kD = ShooterConstants.kLeftSlot_kD;
    leftConfigs.Slot0.kA = ShooterConstants.kLeftSlot_kA;
    leftConfigs.Slot0.kS = ShooterConstants.kLeftSlot_kS;
    leftConfigs.Slot0.kV = ShooterConstants.kLeftSlot_kV;

    left.getConfigurator().apply(leftConfigs);
    right.getConfigurator().apply(rightConfigs);

    // Signals
    leftPosition = left.getPosition();
    leftVelocity = left.getVelocity();
    leftAppliedVolts = left.getMotorVoltage();
    leftCurrent = left.getTorqueCurrent();

    rightPosition = right.getPosition();
    rightVelocity = right.getVelocity();
    rightAppliedVolts = right.getMotorVoltage();
    rightCurrent = right.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ShooterConstants.kStatusUpdateFrequency,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftCurrent,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightCurrent);

    ParentDevice.optimizeBusUtilizationForAll(left, right);
  }

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftCurrent,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightCurrent);

    inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
    inputs.leftVelocityRotPerSec = leftVelocity.getValueAsDouble();
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();

    inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.rightVelocityRotPerSec = rightVelocity.getValueAsDouble();
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
  }

  @Override
  public void setMotorVoltage(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
  }

  @Override
  public void setMotorVoltage(double volts) {
    left.setVoltage(volts);
    right.setControl(new Follower(ShooterConstants.kLeftMotorID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void setVelocityControl(double leftRotPerSec, double rightRotPerSec) {
    left.setControl(m_velocityLeft.withVelocity(leftRotPerSec));
    right.setControl(m_velocityRight.withVelocity(rightRotPerSec));
  }

  @Override
  public void setVelocityControl(double rotPerSec) {
    left.setControl(m_velocityLeft.withVelocity(rotPerSec));
    right.setControl(new Follower(ShooterConstants.kLeftMotorID, MotorAlignmentValue.Opposed));
  }
}
