package frc.robot.constants;

public final class TurretConstants {
  private TurretConstants() {}

  // CANbus name
  public static final String kCANBusName = "rio";

  // Motor and sensor IDs
  public static final int kTurretMotorID = 42;
  public static final int kPigeonId = 2;

  // Motor configuration
  public static final boolean kMotorNeutralCoast = true;
  public static final boolean kMotorInvertCCWPositive = true;

  // Control tuning
  public static final double kStatusUpdateFrequency = 50.0;
  public static final double kSlot_kP = 0.5;
  public static final double kSlot_kI = 0.0;
  public static final double kSlot_kD = 0.0;
  public static final double kSlot_kS = 0.0;
  public static final double kSlot_kV = 0.0;

  public static final double kToleranceDeg = 0.0;

  public static final double gearRatio = 0.0;
}
