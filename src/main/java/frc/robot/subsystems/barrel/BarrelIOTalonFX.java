package frc.robot.subsystems.barrel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import frc.robot.constants.BarrelConstants;

public class BarrelIOTalonFX implements BarrelIO {
  // Hardware objects
  private final TalonFX barrel;
  private TalonFXConfiguration barrelConfigs = new TalonFXConfiguration();

  private final LinearSystemLoop<N1, N1, N1> m_loop;

  // Inputs from motor
  private final StatusSignal<Angle> BarrelPosition;
  private final StatusSignal<AngularVelocity> BarrelVelocity;
  private final StatusSignal<Voltage> BarrelAppliedVolts;
  private final StatusSignal<Current> BarrelCurrent;

  public BarrelIOTalonFX() {
    barrel = new TalonFX(BarrelConstants.barrelID, "rio");

    LinearSystem<N1, N1, N1> barrelSystem =
        LinearSystemId.identifyVelocitySystem(
            BarrelConstants.kLeftSlot_kV, BarrelConstants.kLeftSlot_kA);

    KalmanFilter<N1, N1, N1> m_observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            barrelSystem,
            VecBuilder.fill(
                BarrelConstants
                    .kKalmanModelStandardDeviation), // How accurate we think our model is
            VecBuilder.fill(
                BarrelConstants
                    .kKalmanEncoderStandardDeviation), // How accurate we think our encoder data is
            BarrelConstants.kLoopTime);

    LinearQuadraticRegulator<N1, N1, N1> LQR =
        new LinearQuadraticRegulator<>(
            barrelSystem,
            VecBuilder.fill(BarrelConstants.kTolerence),
            VecBuilder.fill(BarrelConstants.kVoltageTolerance),
            BarrelConstants.kLoopTime);

    m_loop = new LinearSystemLoop<>(barrelSystem, LQR, m_observer,BarrelConstants.barrelMaxVoltage, BarrelConstants.kLoopTime);

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
    inputs.BarrelPositionRad = Units.rotationsToRadians(BarrelPosition.getValueAsDouble());
    inputs.BarrelVelocityRadPerSec = Units.rotationsToRadians(BarrelVelocity.getValueAsDouble());
    inputs.BarrelAppliedVolts = BarrelAppliedVolts.getValueAsDouble();
    inputs.BarrelCurrentAmps = BarrelCurrent.getValueAsDouble();
  }

  @Override
  public void setBarrelVoltage(double volts) {
    barrel.setVoltage(volts);
  }

  @Override
  public void stop() {
    barrel.setVoltage(0);
  }

  @Override
  public void updateOutputs(BarrelIO.BarrelIOInputs inputs, double targetVelocity) {
    m_loop.setNextR(VecBuilder.fill(targetVelocity));
    m_loop.correct(VecBuilder.fill(inputs.BarrelVelocityRadPerSec));

    m_loop.predict(BarrelConstants.kLoopTime);

    // Apply LQR control to the left motor
    this.setMotorVoltage(m_loop.getU(0));
  }

  @Override
  public void idle(BarrelIO.BarrelIOInputs inputs) {
    if (inputs.BarrelVelocityRadPerSec > BarrelConstants.kIdleSpeed + BarrelConstants.kTolerence) {
      stop();

    } else {
      updateOutputs(inputs, BarrelConstants.kIdleSpeed);
    }
  }

  @Override
  public void setMotorVoltage(double volts) {

    barrel.setVoltage(volts);
  }
}
