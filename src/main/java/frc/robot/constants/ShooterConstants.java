package frc.robot.constants;

public final class ShooterConstants {
  private ShooterConstants() {}

  // CAN IDs
  public static final int kLeftMotorID = 44;
  public static final int kRightMotorID = 45;
  public static final int kPigeonID = 6;

  // Motor configuration
  public static final boolean kMotorNeutralCoast = true;
  public static final boolean kMotorInvertLeftCCWPositive = true;
  public static final boolean kMotorInvertRightCCWPositive = true;

  // Control slots / tuning
  public static final double kStatusUpdateFrequency = 50.0;

  public static final double kLeftSlot_kP = 0.0;
  public static final double kLeftSlot_kI = 0.0;
  public static final double kLeftSlot_kD = 0.0;
  public static final double kLeftSlot_kA = 0.0;
  public static final double kLeftSlot_kS = 0.0;
  public static final double kLeftSlot_kV = 0.0;

  public static final double kRightSlot_kP = 0.0;
  public static final double kRightSlot_kI = 0.0;
  public static final double kRightSlot_kD = 0.0;
  public static final double kRightSlot_kA = 0.0;
  public static final double kRightSlot_kS = 0.0;
  public static final double kRightSlot_kV = 0.0;

  public static final double kTolerence = 0.0;

  // Voltage Constants
  public static final double kShootingVoltage = 0.0;

  // Velocity Constants
  public static final double kShootingSpeed = 0.0;
  public static final double kIdleSpeed = 0.0;
}
