package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX right;

  private final LinearSystemLoop<N1, N1, N1> m_loop;
  private final Matrix<N1, N1> shooterRightState = VecBuilder.fill(0.0);

  // Status signals for telemetry and odometry

  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightCurrent;

  public ShooterIOTalonFX() {
    // Initialize hardware on the RIO CAN bus
    right = new TalonFX(ShooterConstants.kRightMotorID, "canivore");

    LinearSystem<N1, N1, N1> flywheelSystem =
        LinearSystemId.identifyVelocitySystem(
            ShooterConstants.kRightSlot_kV, ShooterConstants.kRightSlot_kA);

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
            VecBuilder.fill(ShooterConstants.kVoltageTolerance),
            ShooterConstants.kLoopTime);

    m_loop =
        new LinearSystemLoop<>(
            flywheelSystem,
            LQR,
            m_observer,
            ShooterConstants.kMaxVoltage,
            ShooterConstants.kLoopTime);

    m_loop.reset(shooterRightState);

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

    // rightConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    rightConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
    rightConfigs.CurrentLimits.SupplyCurrentLimit = 80.0;

    // Apply PID and Feedforward gains

    right.getConfigurator().apply(rightConfigs);

    rightPosition = right.getPosition();
    rightVelocity = right.getVelocity();
    rightAppliedVolts = right.getMotorVoltage();
    rightCurrent = right.getTorqueCurrent();

    // Optimize CAN bus usage
    BaseStatusSignal.setUpdateFrequencyForAll(
        ShooterConstants.kStatusUpdateFrequency,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightCurrent);

    ParentDevice.optimizeBusUtilizationForAll(right);
  }

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(rightPosition, rightVelocity, rightAppliedVolts, rightCurrent);

    inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
  }

  @Override
  public void updateOutputs(ShooterIO.ShooterIOInputs inputs, double targetVelocity) {
    shooterRightState.set(0, 0, inputs.rightVelocityRadPerSec);
    m_loop.setNextR(VecBuilder.fill(targetVelocity));
    m_loop.correct(VecBuilder.fill(inputs.rightVelocityRadPerSec));

    m_loop.predict(ShooterConstants.kLoopTime);

    // Apply LQR control to the left motor
    this.setMotorVoltage(m_loop.getU(0));
  }

  @Override
  public void idle(ShooterIO.ShooterIOInputs inputs) {

    if (inputs.rightVelocityRadPerSec > ShooterConstants.kIdleSpeed + ShooterConstants.kTolerence) {
      stop();
    } else {
      // updateOutputs(inputs, ShooterConstants.kIdleSpeed);
    }
  }

  @Override
  public void setMotorVoltage(double volts) {

    // right.setVoltage(volts);

    SmartDashboard.putNumber("LQR/supplyVolts", right.getSupplyVoltage(true).getValueAsDouble());
    right.setControl(
        new VoltageOut(
            MathUtil.clamp(volts, volts, right.getSupplyVoltage(true).getValueAsDouble())));
    SmartDashboard.putNumber("LQR/volts", volts);
    SmartDashboard.putNumber("LQR/error", m_loop.getError(0));
    SmartDashboard.putNumber("LQR/u", m_loop.getU(0));
    SmartDashboard.putNumber("LQR/nextR", m_loop.getNextR(0));
    SmartDashboard.putNumber("LQR/xhat", m_loop.getXHat(0));
    SmartDashboard.putNumber("LQR/uff", m_loop.getFeedforward().getUff(0));
  }

  public void stop() {
    right.setVoltage(0);
  }
}
