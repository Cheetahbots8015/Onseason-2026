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
  public static final double kStatusUpdateFrequency = 50.0;

  public static final double kLeftSlot_kP = 0.0;
  public static final double kLeftSlot_kI = 0.0;
  public static final double kLeftSlot_kD = 0.0;
  public static final double kLeftSlot_kA = 0.0013901;
  public static final double kLeftSlot_kS = 0.0;
  public static final double kLeftSlot_kV = 0.018461;

  public static final double kRightSlot_kP = 0.0;
  public static final double kRightSlot_kI = 0.0;
  public static final double kRightSlot_kD = 0.0;
  public static final double kRightSlot_kA = 0.0;
  public static final double kRightSlot_kS = 0.0;
  public static final double kRightSlot_kV = 0.0;

  public static final double kTolerence = 3;

  public static final double kMaxVoltage = 12; // Maximum voltage for the motors

  // Voltage Constants
  public static final double kShootingVoltage = 0.0;

  // Velocity Constants
  public static final double kShootingSpeed = 30.0;
  public static final double kIdleSpeed = 10.0;

  // Loop timing
  public static final double kLoopTime = 0.02; // 20 ms

  // Kalman filter constants
  public static final double kKalmanModelStandardDeviation =
      3.0; // How accurate we think our model is
  public static final double kKalmanEncoderStandardDeviation =
      3.0; // How accurate we think our encoder data is
}
