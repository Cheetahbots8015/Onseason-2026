package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
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

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX left;
  private final TalonFX right;
  private final Pigeon2 pigeon;

  private final VelocityVoltage m_velocityLeft = new VelocityVoltage(0).withSlot(0);

  private final Follower m_follower =
      new Follower(ShooterConstants.kLeftMotorID, MotorAlignmentValue.Aligned);

  // Status signals for telemetry and odometry
  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftCurrent;

  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightCurrent;

  public ShooterIOTalonFX() {
    // Initialize hardware on the RIO CAN bus
    left = new TalonFX(ShooterConstants.kLeftMotorID, "canivore");
    right = new TalonFX(ShooterConstants.kRightMotorID, "canivore");
    pigeon = new Pigeon2(ShooterConstants.kPigeonID, "canivore");

    TalonFXConfiguration leftConfigs = new TalonFXConfiguration();
    TalonFXConfiguration rightConfigs = new TalonFXConfiguration();

    // Configure neutral mode
    NeutralModeValue neutralMode =
        ShooterConstants.kMotorNeutralCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    leftConfigs.MotorOutput.NeutralMode = neutralMode;
    rightConfigs.MotorOutput.NeutralMode = neutralMode;

    // Configure motor inversions
    leftConfigs.MotorOutput.Inverted =
        ShooterConstants.kMotorInvertLeftCCWPositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    rightConfigs.MotorOutput.Inverted =
        ShooterConstants.kMotorInvertRightCCWPositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    // Apply PID and Feedforward gains
    leftConfigs.Slot0.kP = ShooterConstants.kLeftSlot_kP;
    leftConfigs.Slot0.kI = ShooterConstants.kLeftSlot_kI;
    leftConfigs.Slot0.kD = ShooterConstants.kLeftSlot_kD;
    leftConfigs.Slot0.kS = ShooterConstants.kLeftSlot_kS;
    leftConfigs.Slot0.kV = ShooterConstants.kLeftSlot_kV;

    rightConfigs.Slot0.kP = ShooterConstants.kRightSlot_kP;
    rightConfigs.Slot0.kI = ShooterConstants.kRightSlot_kI;
    rightConfigs.Slot0.kD = ShooterConstants.kRightSlot_kD;
    rightConfigs.Slot0.kS = ShooterConstants.kRightSlot_kS;
    rightConfigs.Slot0.kV = ShooterConstants.kRightSlot_kV;

    left.getConfigurator().apply(leftConfigs);
    right.getConfigurator().apply(rightConfigs);

    // Setup status signals
    leftPosition = left.getPosition();
    leftVelocity = left.getVelocity();
    leftAppliedVolts = left.getMotorVoltage();
    leftCurrent = left.getTorqueCurrent();

    rightPosition = right.getPosition();
    rightVelocity = right.getVelocity();
    rightAppliedVolts = right.getMotorVoltage();
    rightCurrent = right.getTorqueCurrent();

    // Optimize CAN bus usage
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
  public void setMotorVoltage(double volts) {
    left.setVoltage(volts);
    right.setControl(m_follower);
  }

  @Override
  public void setVelocityControl(double rotPerSec) {
    left.setControl(m_velocityLeft.withVelocity(rotPerSec));
    right.setControl(m_follower);
  }
}
