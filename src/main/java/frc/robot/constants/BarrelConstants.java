package frc.robot.constants;

public class BarrelConstants {
  public static final int barrelID = 33;

  public static final boolean barrel_neutralmode_Coast = true;
  public static final boolean barrel_inverted_CounterClockwisePositive = true;

  // public static final double reductionRatio = 1.0 / 3.0;
  public static final double statusUpdateFrequency = 50.0;

  public static final double barrelkP = 0.25;
  public static final double barrelkI = 0.0;
  public static final double barrelkD = 0.0;
  public static final double barrelkS = 0.0;
  public static final double barrelkV = 0.110;
  public static final double barrelkA = 0.0;

  public static final double statorCurrentLimit = 0;

  public static final double supplyCurrentLimit = 25;

  public static final boolean statorCurrentLimitEnable = false;

  public static final boolean supplyCurrentLimitEnable = false;

  public static final double kLeftSlot_kV = 0;

  public static final double kLeftSlot_kA = 0;

  public static final double kKalmanModelStandardDeviation = 3.0;

  public static final double kKalmanEncoderStandardDeviation = 99999.0;

  public static final double kLoopTime = 0.02;

  public static final double kVoltageTolerance = 12;
  public static final double kTolerence = 3;

  public static final double kIdleSpeed = 0;

  public static final double BarrelReverseVelocity = 0;

  public static final double BarrelForwardVelocity = 0;

  public static final double barrelMaxVoltage = 5.0;
}
