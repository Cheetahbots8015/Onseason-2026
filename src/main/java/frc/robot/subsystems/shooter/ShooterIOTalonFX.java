package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX left;
  private final TalonFX right;

  private final Matrix<N1, N1> shooterRightState = VecBuilder.fill(0.0);

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
    final VelocityVoltage leftVelocityVoltage = new VelocityVoltage(null).withSlot(0);
    final VelocityVoltage rightVelocityVoltage = new VelocityVoltage(null).withSlot(0);

    right = new TalonFX(ShooterConstants.kRightMotorID, "canivore");
    left = new TalonFX(ShooterConstants.kLeftMotorID, "canivore");

    

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
      
    leftConfigs.Slot0.kP = ShooterConstants.kLeftSlot_kP;
    leftConfigs.Slot0.kI = ShooterConstants.kLeftSlot_kI;
    leftConfigs.Slot0.kD = ShooterConstants.kLeftSlot_kD;
    leftConfigs.Slot0.kA = ShooterConstants.kLeftSlot_kA;
    leftConfigs.Slot0.kS = ShooterConstants.kLeftSlot_kS;
    leftConfigs.Slot0.kV = ShooterConstants.kLeftSlot_kV;

    rightConfigs.Slot0.kP = ShooterConstants.kRightSlot_kP;
    rightConfigs.Slot0.kI = ShooterConstants.kRightSlot_kI;
    rightConfigs.Slot0.kD = ShooterConstants.kRightSlot_kD;
    rightConfigs.Slot0.kA = ShooterConstants.kRightSlot_kA;
    rightConfigs.Slot0.kS = ShooterConstants.kRightSlot_kS;
    rightConfigs.Slot0.kV = ShooterConstants.kRightSlot_kV;


    // rightConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    rightConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
    rightConfigs.CurrentLimits.SupplyCurrentLimit = 90.0;

    // Apply PID and Feedforward gains
    left.getConfigurator().apply(leftConfigs);
    right.getConfigurator().apply(rightConfigs);

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
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightCurrent,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftCurrent);

    ParentDevice.optimizeBusUtilizationForAll(left,right);
  }

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(rightPosition, rightVelocity, rightAppliedVolts, rightCurrent,leftPosition, leftVelocity, leftAppliedVolts, leftCurrent);

    inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
    
    inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
    inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();
  }

  @Override
  public void updateOutputs(ShooterIO.ShooterIOInputs inputs, double targetVelocity) {
  }

  @Override
  public void idle(ShooterIO.ShooterIOInputs inputs) {
  }

  @Override
  public void setleftMotorVoltage(double volts) {
    left.setVoltage(volts);
  }


  @Override
  public void setrightMotorVoltage(double volts) {
    right.setVoltage(volts);
  }

  public void stop() {
    left.setVoltage(0);
    right.setVoltage(0);
  }

  @Override
  public void setLeftVelocityVoltage(double velocity){
    left.setControl(leftVelocityVoltage.withVelocity(velocity));
  }
}
