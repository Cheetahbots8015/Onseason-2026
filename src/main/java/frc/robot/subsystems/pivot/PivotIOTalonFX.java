package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.pivot.PivotIO.PivotIOInputs;
import frc.robot.util.CheetahUtil;

public class PivotIOTalonFX implements PivotIO {
  // Hardware objects
  private final TalonFX pivot;
  private TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
  // Voltage control requests
  final MotionMagicVoltage m_motion = new MotionMagicVoltage(0).withSlot(1);
  // Inputs from pivot
  private final StatusSignal<Angle> Position;
  private final StatusSignal<AngularVelocity> Velocity;
  private final StatusSignal<Voltage> AppliedVolts;
  private final StatusSignal<Current> Current;

  public PivotIOTalonFX() {
    pivot = new TalonFX(PivotConstants.pivotID, "canivore");
    pivotConfigs.MotorOutput.withNeutralMode(
        PivotConstants.neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    pivotConfigs.MotorOutput.withInverted(
        PivotConstants.inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    // Set PID and feedforward constants from constants file
    pivotConfigs.Slot1.kP = PivotConstants.kPMM;
    pivotConfigs.Slot1.kI = PivotConstants.kIMM;
    pivotConfigs.Slot1.kD = PivotConstants.kDMM;
    pivotConfigs.Slot1.kG = PivotConstants.kGMM;
    pivotConfigs.Slot1.kV = PivotConstants.kVMM;

    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        -0.075 / PivotConstants.ReductionRatio;
    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        0.5 / PivotConstants.ReductionRatio; // 0.5 cycle, 106.66667 reduction ratio
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = 76.8;
    pivotConfigs.MotionMagic.MotionMagicAcceleration = 230.4;
    // Apply the configuration to the motor
    pivot.getConfigurator().apply(pivotConfigs);

    // Create drive status signals
    Position = pivot.getPosition();
    Velocity = pivot.getVelocity();
    AppliedVolts = pivot.getMotorVoltage();
    Current = pivot.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        PivotConstants.statusUpdateFrequency, Velocity, AppliedVolts, Current, Position);
    ParentDevice.optimizeBusUtilizationForAll(pivot);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(Position, Velocity, AppliedVolts, Current);
    // Update pivot inputs
    inputs.PositionRad = Units.rotationsToRadians(Position.getValueAsDouble());
    inputs.VelocityRadPerSec = Units.rotationsToRadians(Velocity.getValueAsDouble());
    inputs.AppliedVolts = AppliedVolts.getValueAsDouble();
    inputs.CurrentAmps = Current.getValueAsDouble();
    inputs.PivotPositionDegree = CheetahUtil.pivotRotationToDegrees(Position.getValueAsDouble());
  }

  @Override
  public void setPivotVoltage(double output) {
    pivot.setVoltage(output);
  }

  @Override
  /**
   * Sets the pivot position using a motion magic control. The position is specified in degrees,
   * which is converted to rotations for the motor.
   *
   * @param degrees The desired position in degrees.
   */
  public void setPosition(double degrees) {
    if (degrees < 0 || degrees > 180) {
      try {
        throw new IllegalArgumentException("Pivot position must be between 0 and 180 degrees.");
      } catch (IllegalArgumentException e) {
        System.out.println("Error: " + e.getMessage());
      }
    }
    double rotation = CheetahUtil.pivotDegreesToRotation(degrees);
    pivot.setControl(m_motion.withPosition(rotation));
  }
}
