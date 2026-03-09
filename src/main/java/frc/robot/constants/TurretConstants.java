package frc.robot.constants;

public final class TurretConstants {
  // Tunable turret constants
  public static double kForwardSoftLimitDeg = 270.0;

  private TurretConstants() {}

  // CANbus name
  public static final String kCANBusName = "canivore";

  // Motor and sensor IDs
  public static final int kTurretMotorID = 49;
  public static final int kPigeonId = 6;

  // Motor configuration
  public static final boolean kMotorNeutralCoast = true;
  public static final boolean kMotorInvertCCWPositive = false;

  // Control tuning
  public static final double kStatusUpdateFrequency = 50.0;
  public static final double kSlot_kP = 2;
  public static final double kSlot_kI = 0.0;
  public static final double kSlot_kD = 0.0;
  public static final double kSlot_kS = 0.0;
  public static final double kSlot_kV = 0.1;

  public static final double kToleranceDeg = 0.0;

  public static final double gearRatio = 1.0 / 40;
}
