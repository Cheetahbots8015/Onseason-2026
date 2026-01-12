package frc.robot.constants;

public class IntakeConstants {
  public static final int indexerID = 49;
  public static final int intakeID = 47;
  public static final int armID = 48;
  public static final double ArmPositionDeadband = 2.0;
  public static final boolean indexer_neutralmode_Coast = true;
  public static final boolean indexer_inverted_CounterClockwisePositive = true;
  public static final boolean intake_neutralmode_Coast = true;
  public static final boolean intake_inverted_CounterClockwisePositive = true;
  public static final boolean arm_neutralmode_Coast = false;
  public static final boolean arm_inverted_CounterClockwisePositive = false;
  public static final double indexer_kP = 0.0;
  public static final double indexer_kI = 0.0;
  public static final double indexer_kD = 0.0;
  public static final double indexer_kA = 0.0;
  public static final double indexer_kS = 0.0;
  public static final double indexer_kV = 0.0;
  public static final double intake_kP = 0.0;
  public static final double intake_kI = 0.0;
  public static final double intake_kD = 0.0;
  public static final double intake_kA = 0.0;
  public static final double intake_kS = 0.0;
  public static final double intake_kV = 0.0;
  public static final double arm_kP = 60;
  public static final double arm_kI = 0.0;
  public static final double arm_kD = 0.0;
  public static final double arm_kA = 0.0;
  public static final double arm_kS = 0.0;
  public static final double arm_kV = 1.0;
  public static final double arm_kG = -30;

  // canRange can ID, name and configs
  public static final int canRangeID = 60;
  public static final String canName = "canivore";
  public static final double canRangeThreshold = 0.06;
  public static final double canRangeHysteresis = 0.005;
  public static final int minSignalStrength = 2500;
  public static final double statusUpdateFrequency = 50.0;
  public static final double PositionDeadband = 5.0;
  public static final double ReductionRatio = 1 / 28.125;
  public static final double armDownDegree = 61.056;
}
