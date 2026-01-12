package frc.robot.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class GetClosetReefCommand extends Command {
  public static Command create(
      Drive drive, Translation2d startTranslation, double startDegrees, int lrIndex) {
    return Commands.runOnce(
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

          Pose2d startPose = new Pose2d(startTranslation, Rotation2d.fromDegrees(startDegrees));

          if (alliance == Alliance.Red) {
            startPose =
                FieldConstants.rotateAroundCenter(
                    startPose, FieldConstants.FieldCenter, Rotation2d.k180deg);
          }

          drive.setPose(startPose);

          // 计算最近的 reef
          Pose2d currentPose = drive.getPose();
          Pose2d closestPose =
              FieldConstants.getClosestReefPose(currentPose.getTranslation(), lrIndex, alliance);

          AutoBuilder.pathfindToPose(
                  closestPose,
                  new PathConstraints(
                      3.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(360)),
                  0.0)
              .schedule();
        });
  }
}
