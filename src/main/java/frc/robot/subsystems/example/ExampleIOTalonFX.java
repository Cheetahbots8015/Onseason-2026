package frc.robot.subsystems.example;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ExampleConstants;

public class ExampleIOTalonFX implements ExampleIO {
  // Hardware objects
  private final TalonFX motor;
  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  // Voltage control requests
  final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(0);

  // Torque current control requests
  final TorqueCurrentFOC m_TorqueCurrentFOC = new TorqueCurrentFOC(0);

  // Inputs from motor
  private final StatusSignal<Angle> MotorPosition;
  private final StatusSignal<AngularVelocity> MotorVelocity;
  private final StatusSignal<Voltage> MotorAppliedVolts;
  private final StatusSignal<Current> MotorCurrent;

  // filter
  private final MedianFilter filter = new MedianFilter(30);

  public ExampleIOTalonFX() {
    motor = new TalonFX(ExampleConstants.motorID, "canivore");
    motorConfigs.MotorOutput.withNeutralMode(
        ExampleConstants.motor_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    motorConfigs.MotorOutput.withInverted(
        ExampleConstants.motor_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // Set PID and feedforward constants from constants file
    motorConfigs.Slot0.kP = ExampleConstants.motorkP;
    motorConfigs.Slot0.kI = ExampleConstants.motorkI;
    motorConfigs.Slot0.kD = ExampleConstants.motorkD;
    motorConfigs.Slot0.kA = ExampleConstants.motorkA;
    motorConfigs.Slot0.kS = ExampleConstants.motorkS;
    motorConfigs.Slot0.kV = ExampleConstants.motorkV;

    // Apply the configuration to the motor
    motor.getConfigurator().apply(motorConfigs);

    // Create drive status signals
    MotorPosition = motor.getPosition();
    MotorVelocity = motor.getVelocity();
    MotorAppliedVolts = motor.getMotorVoltage();
    MotorCurrent = motor.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ExampleConstants.statusUpdateFrequency,
        MotorPosition,
        MotorVelocity,
        MotorAppliedVolts,
        MotorCurrent);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(ExampleIOInputs inputs) {
    BaseStatusSignal.refreshAll(MotorPosition, MotorVelocity, MotorAppliedVolts, MotorCurrent);
    // Update motor inputs
    inputs.MotorPositionRad = Units.rotationsToRadians(MotorPosition.getValueAsDouble());
    inputs.MotorVelocityRadPerSec = Units.rotationsToRadians(MotorVelocity.getValueAsDouble());
    inputs.MotorAppliedVolts = MotorAppliedVolts.getValueAsDouble();
    inputs.MotorCurrentAmps = MotorCurrent.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double motorOutput) {
    motor.setControl(new DutyCycleOut(motorOutput));
  }

  @Override
  public void setMotorVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void MotorVelocityVoltage(double velocity) {
    motor.setControl(m_velocity.withVelocity(velocity));
  }

  @Override
  public void MotorTorqueCurrent(double current) {
    motor.setControl(m_TorqueCurrentFOC.withOutput(current));
  }
}
