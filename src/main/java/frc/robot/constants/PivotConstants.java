package frc.robot.constants;

public class PivotConstants {
  public static final int pivotID = 38;

  public static final double L1Position = 0.0;
  public static final double L2Position = 0.0;
  public static final double L3Position = 0.0;
  public static final double L4Position = 0.0;

  public static final boolean neutralmode_Coast = true;
  public static final boolean inverted_CounterClockwisePositive = false;
  public static final double PositionDeadband = 3.00;
  public static final double kPMM = 0.6;
  public static final double kIMM = 0.0;
  public static final double kDMM = 0.0;
  public static final double kGMM = 0.0;
  public static final double kVMM = 0.1;
  public static final double holdVoltage = 0.2;
  public static final double setPositionUpVoltage = 2;
  public static final double setPositionDownvoltage = -2;
  public static final double dutyCycleDeadband = 0.05;
  public static final double statusUpdateFrequency = 50.0;

  public static final double ClockWiseValue = 0.2;
  public static final double AntiClockWiseValue = -0.2;

  public static final double ReductionRatio = (1.0 / 8.0 * 18 / 64 * 24 / 90);
}
