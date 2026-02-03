package frc.robot.constants;

public class FeederConstants {
  public static final int feederID = 3;

  public static final boolean feeder_neutralmode_Coast = true;
  public static final boolean feeder_inverted_CounterClockwisePositive = false;

  public static final double dutyCycleDeadband = 0.05;
  public static final double statusUpdateFrequency = 50.0;

  public static final double feederkP = 0.00;
  public static final double feederkI = 0.00;
  public static final double feederkD = 0.00;
  public static final double feederkA = 0.00;
  public static final double feederkS = 0.00;
  public static final double feederkV = 0.00;

  public static final double statorCurrentLimit = 40;

  public static final double supplyCurrentLimit = 40;

  public static final boolean statorCurrentLimitEnable = true;

  public static final boolean supplyCurrentLimitEnable = true;
}
