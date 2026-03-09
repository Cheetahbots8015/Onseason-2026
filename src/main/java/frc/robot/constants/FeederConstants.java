package frc.robot.constants;

public class FeederConstants {
  public static final int feederID = 36;

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

  public static final double feederForwardVelocity = 40;

  public static final double feederReverseVelocity = -20;
}
