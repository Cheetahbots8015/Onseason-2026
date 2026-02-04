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
  public static final double barrelkV = 0.112;
  public static final double barrelkA = 0.0;

  public static final double statorCurrentLimit = 0;

  public static final double supplyCurrentLimit = 0;

  public static final boolean statorCurrentLimitEnable = false;

  public static final boolean supplyCurrentLimitEnable = false;

  public static final double BarrelForwardVelocity = -50;
  public static final double BarrelReverseVelocity = 25;
}
