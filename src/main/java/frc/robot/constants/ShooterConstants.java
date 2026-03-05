package frc.robot.constants;

public final class ShooterConstants {
  private ShooterConstants() {}

  // CAN IDs
  public static final int kLeftMotorID = 44;
  public static final int kRightMotorID = 45;

  // Motor configuration
  public static final boolean kMotorNeutralCoast = true;
  public static final boolean kMotorInvertLeftCCWPositive = true;
  public static final boolean kMotorInvertRightCCWPositive = true;

  // Control slots / tuning
  public static final double kStatusUpdateFrequency = 200;

  public static final double kLeftSlot_kP = 0.0;
  public static final double kLeftSlot_kI = 0.0;
  public static final double kLeftSlot_kD = 0.0;
  public static final double kLeftSlot_kA = 0.056987;
  public static final double kLeftSlot_kS = 0.0;
  public static final double kLeftSlot_kV = 0.023537;

  public static final double kRightSlot_kP = 0.0;
  public static final double kRightSlot_kI = 0.0;
  public static final double kRightSlot_kD = 0.0;
  // public static final double kRightSlot_kA = 0.00430785;
  public static final double kRightSlot_kA = 0.0068918;
  public static final double kRightSlot_kS = 0.0;
  // public static final double kRightSlot_kV = 0.0310575;
  public static final double kRightSlot_kV = 0.031003;

  public static final double kTolerence = 1;

  public static final double kMaxVoltage = 9; // Maximum voltage for the motors

  // Voltage Constants
  public static final double kShootingVoltage = 0.0;

  // Velocity Constants
  public static final double kShootingSpeed = 235;
  public static final double kIdleSpeed = 20.0;

  // Loop timing
  public static final double kLoopTime = 0.02; // 20 ms

  // Kalman filter constants
  public static final double kKalmanModelStandardDeviation =
      3.0; // How accurate we think our model is
  public static final double kKalmanEncoderStandardDeviation =
      0.15; // How accurate we think our encoder data is
  public static final double kVoltageTolerance = 16.0;
}
