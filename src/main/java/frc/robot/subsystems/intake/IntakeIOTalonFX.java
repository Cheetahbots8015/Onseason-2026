package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
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
import frc.robot.constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  // Hardware objects
  private final TalonFX flywheel;
  private TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration();
  private final TalonFX arm;
  private TalonFXConfiguration armConfigs = new TalonFXConfiguration();
  private final CANcoder sensor = new CANcoder(IntakeConstants.sensorID, "canivore");

  private final LinearSystemLoop<N1, N1, N1> m_loop;
  private final Matrix<N1, N1> flyWheelState = VecBuilder.fill(0.0);

  final MotionMagicVoltage m_armRequest = new MotionMagicVoltage(0).withSlot(0);
  // Inputs from flywheel
  private final StatusSignal<Angle> FlywheelPosition;
  private final StatusSignal<AngularVelocity> FlywheelVelocity;
  private final StatusSignal<Voltage> FlywheelAppliedVolts;
  private final StatusSignal<Current> FlywheelCurrent;

  private final StatusSignal<Angle> ArmPosition;
  private final StatusSignal<AngularVelocity> ArmVelocity;
  private final StatusSignal<Voltage> ArmAppliedVolts;
  private final StatusSignal<Current> ArmCurrent;

  private final StatusSignal<Angle> SensorDegrees;

  public IntakeIOTalonFX() {
    flywheel = new TalonFX(IntakeConstants.flywheelID, "canivore");

    LinearSystem<N1, N1, N1> flywheelSystem =
        LinearSystemId.identifyVelocitySystem(
            IntakeConstants.kRightSlot_kV, IntakeConstants.kRightSlot_kA);

    KalmanFilter<N1, N1, N1> m_observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            flywheelSystem,
            VecBuilder.fill(
                IntakeConstants
                    .kKalmanModelStandardDeviation), // How accurate we think our model is
            VecBuilder.fill(
                IntakeConstants
                    .kKalmanEncoderStandardDeviation), // How accurate we think our encoder data is
            IntakeConstants.kLoopTime);


    flywheelConfigs.MotorOutput.withNeutralMode(
        IntakeConstants.flywheel_neutralmode_Coast
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);

    LinearQuadraticRegulator<N1, N1, N1> LQR =
        new LinearQuadraticRegulator<>(
            flywheelSystem,
            VecBuilder.fill(IntakeConstants.kTolerence),
            VecBuilder.fill(IntakeConstants.kVoltageTolerance),
            IntakeConstants.kLoopTime);

    m_loop =
        new LinearSystemLoop<>(
            flywheelSystem,
            LQR,
            m_observer,
            IntakeConstants.kMaxVoltage,
            IntakeConstants.kLoopTime);

    m_loop.reset(flyWheelState);

    flywheelConfigs.CurrentLimits.withSupplyCurrentLimit(60);
    flywheelConfigs.CurrentLimits.withSupplyCurrentLimitEnable(true);

    // Set motor inversion based on desired rotation direction
    flywheelConfigs.MotorOutput.withInverted(
        IntakeConstants.flywheel_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // Set PID and feedforward constants from constants file
    flywheelConfigs.Slot0.kP = IntakeConstants.flywheelkP;
    flywheelConfigs.Slot0.kI = IntakeConstants.flywheelkI;
    flywheelConfigs.Slot0.kD = IntakeConstants.flywheelkD;
    flywheelConfigs.Slot0.kA = IntakeConstants.flywheelkA;
    flywheelConfigs.Slot0.kS = IntakeConstants.flywheelkS;
    flywheelConfigs.Slot0.kV = IntakeConstants.flywheelkV;

    // add ramp
    flywheelConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

    arm = new TalonFX(IntakeConstants.armID, "canivore");
    armConfigs.MotorOutput.withNeutralMode(
        IntakeConstants.arm_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    armConfigs.MotorOutput.withInverted(
        IntakeConstants.arm_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    armConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 43;
    armConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    // Set PID and feedforward constants from constants file
    armConfigs.Slot0.kP = IntakeConstants.armkP;
    armConfigs.Slot0.kI = IntakeConstants.armkI;
    armConfigs.Slot0.kD = IntakeConstants.armkD;
    armConfigs.Slot0.kA = IntakeConstants.armkA;
    armConfigs.Slot0.kS = IntakeConstants.armkS;
    armConfigs.Slot0.kV = IntakeConstants.armkV;
    armConfigs.Slot0.kG = IntakeConstants.armkG;

    armConfigs.MotionMagic.MotionMagicCruiseVelocity = 40;
    armConfigs.MotionMagic.MotionMagicAcceleration = 160;

    // Apply the configuration to the motor
    flywheel.getConfigurator().apply(flywheelConfigs);
    arm.getConfigurator().apply(armConfigs);

    // Create drive status signals
    FlywheelPosition = flywheel.getPosition();
    FlywheelVelocity = flywheel.getVelocity();
    FlywheelAppliedVolts = flywheel.getMotorVoltage();
    FlywheelCurrent = flywheel.getTorqueCurrent();

    ArmPosition = arm.getPosition();
    ArmVelocity = arm.getVelocity();
    ArmAppliedVolts = arm.getMotorVoltage();
    ArmCurrent = arm.getTorqueCurrent();

    // Create status signals for the sensor
    SensorDegrees = sensor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.statusFastUpdateFrequency, FlywheelAppliedVolts, FlywheelCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.statusrRegularUpdateFrequency,
        FlywheelPosition,
        ArmPosition,
        ArmVelocity,
        ArmAppliedVolts,
        ArmCurrent,
        SensorDegrees);
    ParentDevice.optimizeBusUtilizationForAll(flywheel);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        FlywheelPosition,
        FlywheelVelocity,
        FlywheelAppliedVolts,
        FlywheelCurrent,
        ArmPosition,
        ArmVelocity,
        ArmAppliedVolts,
        ArmCurrent,
        SensorDegrees);
    // Update motor inputs
    inputs.FlywheelPositionRad = Units.rotationsToRadians(FlywheelPosition.getValueAsDouble());
    inputs.FlywheelVelocityRadPerSec =
        Units.rotationsToRadians(FlywheelVelocity.getValueAsDouble());
    inputs.FlywheelAppliedVolts = FlywheelAppliedVolts.getValueAsDouble();
    inputs.FlywheelCurrentAmps = FlywheelCurrent.getValueAsDouble();

    inputs.ArmPositionRad = Units.rotationsToRadians(ArmPosition.getValueAsDouble());
    inputs.ArmVelocityRadPerSec = Units.rotationsToRadians(ArmVelocity.getValueAsDouble());
    inputs.ArmAppliedVolts = ArmAppliedVolts.getValueAsDouble();
    inputs.ArmCurrentAmps = ArmCurrent.getValueAsDouble();
    inputs.SensorDegrees = SensorDegrees.getValueAsDouble();
  }

  @Override
  public void updateOutputs(IntakeIO.IntakeIOInputs inputs, double targetVelocity) {
    flyWheelState.set(0, 0, inputs.FlywheelVelocityRadPerSec);
    m_loop.setNextR(VecBuilder.fill(targetVelocity));
    m_loop.correct(VecBuilder.fill(inputs.FlywheelVelocityRadPerSec));

    m_loop.predict(IntakeConstants.kLoopTime);

    // Apply LQR control to the left motor
    this.setFlyWheelVoltage(m_loop.getU(0));
  }


  
  @Override
  public void setFlyWheelVoltage(double volts) {
    SmartDashboard.putNumber("LQR/flywheel_supplyVolts", flywheel.getSupplyVoltage(true).getValueAsDouble());
    flywheel.setControl(
        new VoltageOut(volts).withEnableFOC(true));
    SmartDashboard.putNumber("LQR/flywheel_volts", volts);
    SmartDashboard.putNumber("LQR/flywheel_error", m_loop.getError(0));
    SmartDashboard.putNumber("LQR/flywheel_u", m_loop.getU(0));
    SmartDashboard.putNumber("LQR/flywheel_nextR", m_loop.getNextR(0));
    SmartDashboard.putNumber("LQR/flywheel_xhat", m_loop.getXHat(0));
    SmartDashboard.putNumber("LQR/flywheel_uff", m_loop.getFeedforward().getUff(0));
  }


  @Override
  public void setArmVoltage(double volts) {
    arm.setVoltage(volts);
  }

  @Override
  public void resetArmPosition() {
    arm.setPosition(0.0);
  }

  @Override
  public void ArmPositionVoltage(double targetPosition) {
    arm.setControl(m_armRequest.withPosition(Units.radiansToRotations(targetPosition)));
  }

  public void flywheelStop() {
    flywheel.setVoltage(0);
  }
}
