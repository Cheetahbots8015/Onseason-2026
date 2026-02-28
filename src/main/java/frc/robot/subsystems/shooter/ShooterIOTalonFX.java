package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX left;
  private final TalonFX right;

  private final Follower m_follower =
      new Follower(ShooterConstants.kLeftMotorID, MotorAlignmentValue.Aligned);

  private final LinearSystemLoop<N1, N1, N1> m_loop;
  private final Matrix<N1, N1> shooterLeftState = VecBuilder.fill(0.0);

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

    LinearSystem<N1, N1, N1> flywheelSystem =
        LinearSystemId.identifyVelocitySystem(
            ShooterConstants.kLeftSlot_kV, ShooterConstants.kLeftSlot_kA);

    KalmanFilter<N1, N1, N1> m_observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            flywheelSystem,
            VecBuilder.fill(
                ShooterConstants
                    .kKalmanModelStandardDeviation), // How accurate we think our model is
            VecBuilder.fill(
                ShooterConstants
                    .kKalmanEncoderStandardDeviation), // How accurate we think our encoder data is
            ShooterConstants.kLoopTime);

    LinearQuadraticRegulator<N1, N1, N1> LQR =
        new LinearQuadraticRegulator<>(
            flywheelSystem,
            VecBuilder.fill(ShooterConstants.kTolerence),
            VecBuilder.fill(ShooterConstants.kMaxVoltage),
            ShooterConstants.kLoopTime);

    m_loop =
        new LinearSystemLoop<>(
            flywheelSystem,
            LQR,
            m_observer,
            ShooterConstants.kMaxVoltage,
            ShooterConstants.kLoopTime);

    // m_loop.reset(shooterLeftState); // Reset the state

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
    inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();

    inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
  }

  @Override
  public void updateOutputs(ShooterIO.ShooterIOInputs inputs, double targetVelocity) {
    shooterLeftState.set(0, 0, inputs.leftVelocityRadPerSec);
    m_loop.setNextR(VecBuilder.fill(targetVelocity));
    m_loop.correct(VecBuilder.fill(inputs.leftVelocityRadPerSec));

    m_loop.predict(ShooterConstants.kLoopTime);

    // Apply LQR control to the left motor
    this.setMotorVoltage(m_loop.getU(0));
  }

  @Override
  public void idle(ShooterIO.ShooterIOInputs inputs) {

    if (inputs.rightVelocityRadPerSec > ShooterConstants.kIdleSpeed + ShooterConstants.kTolerence) {
      stop();
      ;
    } else {
      updateOutputs(inputs, ShooterConstants.kIdleSpeed);
    }
  }

  @Override
  public void setMotorVoltage(double volts) {

    right.setVoltage(volts);
    // left.setControl(m_follower);
  }

  public void stop() {
    right.setVoltage(0);
    ;
  }
}
