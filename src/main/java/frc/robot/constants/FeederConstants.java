package frc.robot.constants;

public class FeederConstants {
  public static final int feederID = 3;

  public static final boolean feeder_neutralmode_Coast = true;
  public static final boolean feeder_inverted_CounterClockwisePositive = false;

  public static final double dutyCycleDeadband = 0.05;
  public static final double statusUpdateFrequency = 50.0;

  public static final double feederkP = 0.15;
  public static final double feederkI = 0.00;
  public static final double feederkD = 0.00;
  public static final double feederkA = 0.00;
  public static final double feederkS = 0.00;
  public static final double feederkV = 0.113;

  public static final double statorCurrentLimit = 0;

  public static final double supplyCurrentLimit = 0;

  public static final boolean statorCurrentLimitEnable = false;

  public static final boolean supplyCurrentLimitEnable = false;

  public static final double feederForwardVoltage = 0.0;

  public static final double feederReverseVoltage = 0.0;

  public static final double feederForwardVelocity = 60;

  public static final double feederReverseVelocity = -20;

  public static final double kTolerence = 3;

  public static final double kMaxVoltage = 12; // Maximum voltage for the motors

  // Loop timing
  public static final double kLoopTime = 0.02; // 20 ms

  // Kalman filter constants
  public static final double kKalmanModelStandardDeviation =
      3.0; // How accurate we think our model is
  public static final double kKalmanEncoderStandardDeviation =
      3.0; // How accurate we think our encoder data is

  public static final double kIdleSpeed = 10.0; // Speed at which the feeder should idle
}
