package frc.robot.constants;

public class HoodConstants {
  public static final int hoodID = 46;

  public static final boolean hood_neutralmode_Coast = false;
  public static final boolean hood_inverted_CounterClockwisePositive = true;

  public static final double reductionRatio = 15.0 / 36.0;
  public static final double statusUpdateFrequency = 50;

  // Units: Degrees  
  public static final double hoodkP = 3.6;
  public static final double hoodkI = 0.0;
  public static final double hoodkD = 0.0;
  public static final double hoodkS = 0.0; 
  public static final double hoodkV = 0.0;
  public static final double hoodkA = 0.0;

  // Units: Rotations 
  public static final double reverseSoftLimitThreshold = 0.012;
  public static final double forwardSoftLimitThreshold = 1.95;

  public static final boolean reverseSoftLimitEnable = true;
  public static final boolean forwardSoftLimitEnable = true;
}
