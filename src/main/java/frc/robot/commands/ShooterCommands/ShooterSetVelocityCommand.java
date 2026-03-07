package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterSetVelocityCommand extends Command {

  private final ShooterSubsystem m_subsystem;
  private final MedianFilter m_filter;
  private double distance;
  private double temp;
  private boolean isNearTrench;

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
    temp =
        m_filter.calculate(
            LimelightHelpers.getTargetPose3d_CameraSpace("limelight-shooter").getZ());
    distance = temp == 0 ? distance : temp;
    SmartDashboard.putNumber("DistanceToTag", distance);
    SmartDashboard.putNumber("PredictedVelocity", Math.sqrt(distance * 28409 + 30473));

    if (distance < 2.0) {
      m_subsystem.setTargetVelocity(SmartDashboard.getNumber("kShootingSpeed", 0));
    } else {
      m_subsystem.setTargetVelocity(MathUtil.clamp((Math.sqrt(distance * 28409 + 30473)), 0, 400));
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
