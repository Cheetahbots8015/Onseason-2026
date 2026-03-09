package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterSetVelocityCommand extends Command {

  private final ShooterSubsystem m_subsystem;
  private final MedianFilter m_filter;
  private double distance;
  private double temp;
  private double shootingSpeedOffset;
  private boolean isNearTrench;
  private boolean isRedAlliance = false;

  private final Translation2d RED_TARGET =
      new Translation2d(
          (12.519177399999998 + 11.3118646) / 2, 4.0346376); // Target for Red alliance
  private final Translation2d BLUE_TARGET =
      new Translation2d((5.229174199999999 + 4.0218614) / 2, 4.0346376); // Target for Blue alliance

  private double calculated_angle = 0.0;
  Translation2d target = new Translation2d(0.0, 0.0); // Placeholder for target translation

  public ShooterSetVelocityCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    m_filter = new MedianFilter(5);
    distance = 0;
    temp = 0;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    /*
    isNearTrench = false;
    for (int i = 0; i < LimelightHelpers.getRawFiducials("limelight-shooter").length; i++) {
      int tagID = LimelightHelpers.getRawFiducials("limelight-shooter")[i].id;
      if (tagID == 5 || tagID == 8 || tagID == 11 || tagID == 2) {
        isNearTrench = true;
        break;
      }
    }
    */
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("FeederForwardVelocity", 40);

    Translation2d currentPose =
        new Translation2d(
            SmartDashboard.getNumberArray("translation", new double[] {0.0, 0.0})[0],
            SmartDashboard.getNumberArray("translation", new double[] {0.0, 0.0})[1]);

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      isRedAlliance = true;
      target = RED_TARGET;
    } else {
      isRedAlliance = false;
      target = BLUE_TARGET;
    }
    temp =
        Math.sqrt(
            Math.pow(target.getX() - currentPose.getX(), 2)
                + Math.pow(target.getY() - currentPose.getY(), 2));
    distance = m_filter.calculate(temp - 0.6036);
    SmartDashboard.putNumber("DistanceToTag", distance);
    SmartDashboard.putNumber("PredictedVelocity", Math.sqrt(distance * 28409 + 30473));

    shootingSpeedOffset = SmartDashboard.getNumber("ShootingSpeedOffset", 0.0);
    if (shootingSpeedOffset > 20) {
      shootingSpeedOffset = 20;
      SmartDashboard.putNumber("ShootingSpeedOffset", 20);
    } else if (shootingSpeedOffset < -20) {
      shootingSpeedOffset = -20;
      SmartDashboard.putNumber("ShootingSpeedOffset", -20);
    }

    if (distance < 2.0) {
      m_subsystem.setTargetVelocity(SmartDashboard.getNumber("kShootingSpeed", 0));
    } else {
      m_subsystem.setTargetVelocity(
          MathUtil.clamp((Math.sqrt(distance * 28409 + 30473)) - 5 + shootingSpeedOffset, 0, 400));
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setIdle(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
