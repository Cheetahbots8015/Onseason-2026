package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoConstants {
  public static final Pose2d rightPrepareToIntakePoint =
      FieldConstants.inversePose2dUsingAlliance(
          new Pose2d(2.5, 1.9, new Rotation2d(Math.toRadians(-126))), Alliance.Blue);
  public static final Pose2d rightStation =
      FieldConstants.inversePose2dUsingAlliance(
          new Pose2d(1.5, 0.7, new Rotation2d(Math.toRadians(-126))), Alliance.Blue);
  public static final Pose2d leftPrepareToIntakePoint =
      FieldConstants.inversePose2dUsingAlliance(
          new Pose2d(2.3, 6.4, new Rotation2d(Math.toRadians(126))), Alliance.Blue);
  public static final Pose2d leftStation =
      FieldConstants.inversePose2dUsingAlliance(
          new Pose2d(1.600, 7.300, new Rotation2d(Math.toRadians(126))), Alliance.Blue);

  public static final double fieldLength = 17.548;
  public static final double filedWidth = 8.052;

  public static final Pose2d reefBlueA = new Pose2d(3.256, 4.192, new Rotation2d(0));
  public static final Pose2d reefBlueB = new Pose2d(3.256, 3.844, new Rotation2d(0));
  public static final Pose2d reefBlueC = new Pose2d(3.743, 3.0368, new Rotation2d(1.0471975));
  public static final Pose2d reefBlueD = new Pose2d(4.026, 2.875, new Rotation2d(1.0471975));
  public static final Pose2d reefBlueE = new Pose2d(4.963, 2.873, new Rotation2d(2.09439));
  public static final Pose2d reefBlueF = new Pose2d(5.251, 3.039, new Rotation2d(2.09439));
  public static final Pose2d reefBlueG = new Pose2d(5.718, 3.844, new Rotation2d(3.1415926));
  public static final Pose2d reefBlueH = new Pose2d(5.718, 4.192, new Rotation2d(3.1415926));
  public static final Pose2d reefBlueI = new Pose2d(5.2472, 5.0151, new Rotation2d(-2.09439));
  public static final Pose2d reefBlueJ = new Pose2d(4.9626, 5.1740, new Rotation2d(-2.09439));
  public static final Pose2d reefBlueK = new Pose2d(4.0160, 5.1768, new Rotation2d(-1.0471975));
  public static final Pose2d reefBlueL = new Pose2d(3.7291, 5.0085, new Rotation2d(-1.0471975));
}
