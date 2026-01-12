package frc.robot.subsystems.claw;

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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.ClawConstants;

public class ClawIOTalonFX implements ClawIO {
  // Hardware objects
  private final TalonFX intake;
  private final TalonFX shooter;
  private TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
  private TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
  private final DigitalInput lightTrigger = new DigitalInput(1);
  // Voltage control requests
  final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(0);

  final TorqueCurrentFOC m_TorqueCurrentFOC = new TorqueCurrentFOC(0);
  // Inputs from intake
  private final StatusSignal<Angle> IntakePosition;
  private final StatusSignal<AngularVelocity> IntakeVelocity;
  private final StatusSignal<Voltage> IntakeAppliedVolts;
  private final StatusSignal<Current> IntakeCurrent;
  private final StatusSignal<Angle> ShooterPosition;
  private final StatusSignal<AngularVelocity> ShooterVelocity;
  private final StatusSignal<Voltage> ShooterAppliedVolts;
  private final StatusSignal<Current> ShooterCurrent;

  // filter
  private final MedianFilter filter = new MedianFilter(30);

  public ClawIOTalonFX() {
    intake = new TalonFX(ClawConstants.intakeID, "canivore");
    shooter = new TalonFX(ClawConstants.shooterID, "canivore");
    intakeConfigs.MotorOutput.withNeutralMode(
        ClawConstants.intake_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    intakeConfigs.MotorOutput.withInverted(
        ClawConstants.intake_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    shooterConfigs.MotorOutput.withNeutralMode(
        ClawConstants.shooter_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    shooterConfigs.MotorOutput.withInverted(
        ClawConstants.shooter_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // Set PID and feedforward constants from constants file
    intakeConfigs.Slot0.kP = ClawConstants.intakekP;
    intakeConfigs.Slot0.kI = ClawConstants.intakekI;
    intakeConfigs.Slot0.kD = ClawConstants.intakekD;
    intakeConfigs.Slot0.kA = ClawConstants.intakekA;
    intakeConfigs.Slot0.kS = ClawConstants.intakekS;
    intakeConfigs.Slot0.kV = ClawConstants.intakekV;

    shooterConfigs.Slot0.kP = ClawConstants.shooterkP;
    shooterConfigs.Slot0.kI = ClawConstants.shooterkI;
    shooterConfigs.Slot0.kD = ClawConstants.shooterkD;
    shooterConfigs.Slot0.kA = ClawConstants.shooterkA;
    shooterConfigs.Slot0.kS = ClawConstants.shooterkS;
    shooterConfigs.Slot0.kV = ClawConstants.shooterkV;

    // Apply the configuration to the motor
    intake.getConfigurator().apply(intakeConfigs);
    shooter.getConfigurator().apply(shooterConfigs);

    // Create drive status signals
    IntakePosition = intake.getPosition();
    IntakeVelocity = intake.getVelocity();
    IntakeAppliedVolts = intake.getMotorVoltage();
    IntakeCurrent = intake.getTorqueCurrent();
    ShooterPosition = shooter.getPosition();
    ShooterVelocity = shooter.getVelocity();
    ShooterAppliedVolts = shooter.getMotorVoltage();
    ShooterCurrent = shooter.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ClawConstants.statusUpdateFrequency,
        IntakePosition,
        IntakeVelocity,
        IntakeAppliedVolts,
        IntakeCurrent,
        ShooterPosition,
        ShooterVelocity,
        ShooterAppliedVolts,
        ShooterCurrent);
    ParentDevice.optimizeBusUtilizationForAll(intake, shooter);
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        IntakePosition,
        IntakeVelocity,
        IntakeAppliedVolts,
        IntakeCurrent,
        ShooterPosition,
        ShooterVelocity,
        ShooterAppliedVolts,
        ShooterCurrent);
    // Update intake inputs
    inputs.IntakePositionRad = Units.rotationsToRadians(IntakePosition.getValueAsDouble());
    inputs.IntakeVelocityRadPerSec = Units.rotationsToRadians(IntakeVelocity.getValueAsDouble());
    inputs.IntakeAppliedVolts = IntakeAppliedVolts.getValueAsDouble();
    inputs.IntakeCurrentAmps = IntakeCurrent.getValueAsDouble();

    inputs.ShooterPositionRad = Units.rotationsToRadians(ShooterPosition.getValueAsDouble());
    inputs.ShooterVelocityRadPerSec = Units.rotationsToRadians(ShooterVelocity.getValueAsDouble());
    inputs.ShooterAppliedVolts = ShooterAppliedVolts.getValueAsDouble();
    inputs.ShooterCurrentAmps = ShooterCurrent.getValueAsDouble();

    inputs.lightTrigger = filter.calculate((!lightTrigger.get()) ? 1.0 : 0.0);
  }

  @Override
  public void setOpenLoop(double output, double shooterOutput) {

    intake.setControl(new DutyCycleOut(output));
  }

  // SysId methods
  @Override
  public void setIntakeVoltage(double volts) {
    intake.setVoltage(volts);
  }

  @Override
  public void setShooterVoltage(double volts) {
    shooter.setVoltage(volts);
  }

  @Override
  public void IntakeVelocityVoltage(double velocity) {
    intake.setControl(m_velocity.withVelocity(velocity).withFeedForward(3.5));
  }

  @Override
  public void ClawIntakeTorqueCurrent(double current) {
    intake.setControl(m_TorqueCurrentFOC.withOutput(current));
  }
}
