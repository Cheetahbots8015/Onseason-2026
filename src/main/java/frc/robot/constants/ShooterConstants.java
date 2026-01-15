package frc.robot.constants;

public class ShooterConstants {
  public static final int shooterID = 21;

  public static final boolean intake_neutralmode_Coast = true;
  public static final boolean intake_inverted_CounterClockwisePositive = true;
  public static final boolean shooter_neutralmode_Coast = true;
  public static final boolean shooter_inverted_CounterClockwisePositive = false;

  public static final double dutyCycleDeadband = 0.05;
  public static final double statusUpdateFrequency = 50.0;

  public static final double shooterkP = 0.14;
  public static final double shooterkI = 0.0;
  public static final double shooterkD = 0.1;
  public static final double shooterkA = 0.0;
  public static final double shooterkS = 0.1;
  public static final double shooterkV = 0.13;
}
