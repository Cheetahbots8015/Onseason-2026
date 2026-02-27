package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.constants.FeederConstants;

public class FeederIOTalonFX implements FeederIO {
  // Hardware objects
  private final TalonFX feeder;
  private TalonFXConfiguration feederConfigs = new TalonFXConfiguration();

  // Voltage control requests
  final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(0);

  // Inputs from motor
  private final StatusSignal<Angle> FeederPosition;
  private final StatusSignal<AngularVelocity> FeederVelocity;
  private final StatusSignal<Voltage> FeederAppliedVolts;
  private final StatusSignal<Current> FeederCurrent;

  private final LinearSystemLoop<N1, N1, N1> m_loop;

  public FeederIOTalonFX() {
    feeder = new TalonFX(FeederConstants.feederID, "rio");
    feederConfigs.MotorOutput.withNeutralMode(
        FeederConstants.feeder_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    feederConfigs.MotorOutput.withInverted(
        FeederConstants.feeder_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // Apply the configuration to the motor
    feeder.getConfigurator().apply(feederConfigs);

    LinearSystem<N1, N1, N1> flywheelSystem =
        LinearSystemId.identifyVelocitySystem(FeederConstants.feederkV, FeederConstants.feederkA);

    KalmanFilter<N1, N1, N1> m_observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            flywheelSystem,
            VecBuilder.fill(
                FeederConstants
                    .kKalmanModelStandardDeviation), // How accurate we think our model is
            VecBuilder.fill(
                FeederConstants
                    .kKalmanEncoderStandardDeviation), // How accurate we think our encoder data is
            FeederConstants.kLoopTime);

    LinearQuadraticRegulator<N1, N1, N1> LQR =
        new LinearQuadraticRegulator<>(
            flywheelSystem,
            VecBuilder.fill(FeederConstants.kTolerence),
            VecBuilder.fill(FeederConstants.kVoltageTolerance),
            FeederConstants.kLoopTime);

    m_loop =
        new LinearSystemLoop<>(
            flywheelSystem,
            LQR,
            m_observer,
            FeederConstants.feederMaxVoltage,
            FeederConstants.kLoopTime);

    // Create drive status signals
    FeederPosition = feeder.getPosition();
    FeederVelocity = feeder.getVelocity();
    FeederAppliedVolts = feeder.getMotorVoltage();
    FeederCurrent = feeder.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        FeederConstants.statusUpdateFrequency,
        FeederPosition,
        FeederVelocity,
        FeederAppliedVolts,
        FeederCurrent);
    ParentDevice.optimizeBusUtilizationForAll(feeder);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(FeederPosition, FeederVelocity, FeederAppliedVolts, FeederCurrent);
    // Update motor inputs
    inputs.FeederPositionRad = Units.rotationsToRadians(FeederPosition.getValueAsDouble());
    inputs.FeederVelocityRadPerSec = Units.rotationsToRadians(FeederVelocity.getValueAsDouble());
    inputs.FeederAppliedVolts = FeederAppliedVolts.getValueAsDouble();
    inputs.FeederCurrentAmps = FeederCurrent.getValueAsDouble();
  }

  @Override
  public void updateOutputs(FeederIOInputs inputs, double targetVelocity) {
    m_loop.setNextR(VecBuilder.fill(targetVelocity));
    m_loop.correct(VecBuilder.fill(inputs.FeederVelocityRadPerSec));

    m_loop.predict(FeederConstants.kLoopTime);

    // Apply LQR control to the left motor
    this.setFeederVoltage(m_loop.getU(0));
  }

  @Override
  public void setFeederVoltage(double volts) {
    feeder.setVoltage(volts);
  }

  @Override
  public void idle(FeederIOInputs inputs) {
    if (inputs.FeederVelocityRadPerSec > FeederConstants.kIdleSpeed + FeederConstants.kTolerence) {
      stop();
    } else {
      updateOutputs(inputs, FeederConstants.kIdleSpeed);
    }
  }

  public void stop() {
    feeder.setVoltage(0);
  }
}
