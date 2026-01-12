package frc.robot.util;

import frc.robot.constants.ClimberConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.PivotConstants;
import java.util.function.DoubleSupplier;

public class CheetahUtil {
  /*
   * This class is a placeholder for utility methods related to the Cheetah robot.
   */

  /**
   * Checks if a value is within a specified tolerance of a target value.
   *
   * @param value The value to check.
   * @param target The target value.
   * @param tolerance The allowable deviation from the target.
   * @return True if the value is within the tolerance, false otherwise.
   */
  public static boolean isNear(double value, double target, double tolerance) {
    return Math.abs(value - target) <= tolerance;
  }

  /**
   * Checks if a value is near a target value using a default tolerance of 0.05.
   *
   * @param value The value to check.
   * @param target The target value.
   * @return True if the value is within the default tolerance, false otherwise.
   */
  public static boolean isNear(double value, double target) {
    return isNear(value, target, 0.05); // Default tolerance of 0.05
  }

  /**
   * Checks if a value is within a percentage-based tolerance of a target value.
   *
   * @param value The value to check.
   * @param target The target value.
   * @param percentage The percentage tolerance (e.g., 0.01 for 1%).
   * @return True if the value is within the percentage tolerance, false otherwise.
   */
  public static boolean isNearPercentage(double value, double target, double percentage) {
    double tolerance = Math.abs(target * percentage);
    return isNear(value, target, tolerance);
  }

  /**
   * Checks if a value is near a target value using a default percentage tolerance of 1%.
   *
   * @param value The value to check.
   * @param target The target value.
   * @return True if the value is within the default percentage tolerance, false otherwise.
   */
  public static boolean isNearPercentage(double value, double target) {
    return isNearPercentage(value, target, 0.01); // Default percentage of 1%
  }

  /**
   * Applies a deadband to the input value. Values within the deadband are set to 0, and values
   * outside the deadband are scaled proportionally.
   *
   * @param value The input value.
   * @param deadband The deadband threshold.
   * @param maxValue The maximum possible value for scaling.
   * @return The adjusted value after applying the deadband.
   */
  public static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0; // Value is within the deadband
    }
    return value;
  }

  /**
   * Applies a deadband to the input value from a DoubleSupplier. Values within the deadband are set
   * to 0, and values outside the deadband are returned as is.
   *
   * @param value The DoubleSupplier providing the input value.
   * @param deadband The deadband threshold.
   * @return The adjusted value after applying the deadband.
   */
  public static double applyDeadband(DoubleSupplier value, double deadband) {
    if (Math.abs(value.getAsDouble()) < deadband) {
      return 0.0; // Value is within the deadband
    }
    return value.getAsDouble();
  }

  /**
   * Converts elevator rotations to meters.
   *
   * @param rotations The number of rotations of the elevator mechanism.
   * @return The equivalent distance in meters based on the gear ratio and conversion factor. The
   *     calculation divides the rotations by 6 (gear ratio) and multiplies by the effective
   *     distance per rotation (0.005 * 30 meters).
   */
  public static double elevatorRotationToMeters(double rotations) {
    return rotations / 6.0 * (0.005 * 30) + 0.362;
  }

  /**
   * Converts meters to elevator rotations.
   *
   * @param meters The distance in meters.
   * @return The equivalent number of rotations based on the gear ratio and conversion factor. The
   *     calculation subtracts the offset (0.362) and reverses the scaling.
   */
  public static double elevatorMetersToRotation(double meters) {
    return (meters - 0.362) * 6.0 / (0.005 * 30);
  }

  /**
   * Converts pivot rotations to degrees.
   *
   * @param rotations The number of rotations of the pivot mechanism.
   * @return The equivalent angle in degrees. The calculation multiplies the rotations by 360 and
   *     divides by the effective ratio, which is derived from the product of (1/8 * 18/64 * 24/90).
   */
  public static double pivotRotationToDegrees(double rotations) {
    return rotations * 360.0 * PivotConstants.ReductionRatio;
  }

  /**
   * Converts degrees to pivot rotations.
   *
   * @param degrees The angle in degrees.
   * @return The equivalent number of rotations. The calculation divides the degrees by 360 and
   *     reverses the effective ratio.
   */
  public static double pivotDegreesToRotation(double degrees) {
    return degrees / 360.0 / PivotConstants.ReductionRatio;
  }

  public static double intakeArmRotationToDegrees(double rotations) {
    return rotations * 360.0 * IntakeConstants.ReductionRatio;
  }

  public static double intakeArmDegreesToRotation(double degrees) {
    return degrees / 360.0 / IntakeConstants.ReductionRatio;
  }

  public static double climberPivotRotationToDegrees(double rotations) {
    return rotations * 360.0 * ClimberConstants.ReductionRatio;
  }

  public static double climberPivotDegreesToRotation(double degrees) {
    return degrees / 360.0 / ClimberConstants.ReductionRatio;
  }
}
