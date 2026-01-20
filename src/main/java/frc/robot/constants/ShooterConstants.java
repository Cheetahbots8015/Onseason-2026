package frc.robot.constants;

public class ShooterConstants {
  public static final int shooterID = 21;
  public static final int hoodID = 20;

  public static final boolean shooter_neutralmode_Coast = true;
  public static final boolean shooter_inverted_CounterClockwisePositive = false;
  public static final boolean hood_neutralmode_Coast = true;
  public static final boolean hood_inverted_CounterClockwisePositive = false;

  public static final double dutyCycleDeadband = 0.05;
  public static final double statusUpdateFrequency = 50.0;

  public static final double shooterkP = 0.2;
  public static final double shooterkI = 0.0;
  public static final double shooterkD = 0.0;
  public static final double shooterkA = 0.01;
  public static final double shooterkS = 0.0;
  public static final double shooterkV = 0.11138;

  public static final double hoodkP = 0.2;
  public static final double hoodkI = 0.0;
  public static final double hoodkD = 0.0;
  public static final double hoodkA = 1.0005;
  public static final double hoodkS = 0.0;
  public static final double hoodkV = 0.46509;
}
