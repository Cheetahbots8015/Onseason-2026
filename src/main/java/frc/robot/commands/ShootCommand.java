package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.BarrelConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.subsystems.barrel.BarrelSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootCommand extends Command {

  private final ShooterSubsystem m_subsystem;
  private final BarrelSubsystem m_barrelsubsystem;
  private final FeederSubsystem m_feedersubsystem;
  private final MedianFilter m_filter;
  private double distance;
  private double temp;
  private double shootingSpeedOffset;
  private boolean isNearTrench;
  private boolean isRedAlliance = false;

  // 小巧思
  private XboxController m_Controller;
  private double xCompensation;
  private double yCompensation;

  private final Translation2d RED_TARGET =
      new Translation2d(
          (12.519177399999998 + 11.3118646) / 2, 4.0346376); // Target for Red alliance
  private final Translation2d BLUE_TARGET =
      new Translation2d((5.229174199999999 + 4.0218614) / 2, 4.0346376); // Target for Blue alliance

  private double calculated_angle = 0.0;
  Translation2d target = new Translation2d(0.0, 0.0); // Placeholder for target translation

  public ShootCommand(
      ShooterSubsystem subsystem,
      BarrelSubsystem barrelsubsystem,
      FeederSubsystem feedersubsystem,
      XboxController controller) {
    m_subsystem = subsystem;
    m_barrelsubsystem = barrelsubsystem;
    m_feedersubsystem = feedersubsystem;
    m_filter = new MedianFilter(3);
    distance = 0;
    temp = 0;

    m_Controller = controller;

    addRequirements(subsystem, m_barrelsubsystem, m_feedersubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("HoodPosition", 100);

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
    xCompensation = m_Controller.getLeftX() * 0.4;
    yCompensation = m_Controller.getLeftY();
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

    m_subsystem.setTargetVelocity(
        MathUtil.clamp((Math.sqrt(distance * 28409 + 30473)) - 20 + shootingSpeedOffset, 0, 400));

    if (MathUtil.isNear(
        MathUtil.clamp((Math.sqrt(distance * 28409 + 30473)) - 20 + shootingSpeedOffset, 0, 400),
        m_subsystem.getMotorVelocity(),
        20)) {
      m_feedersubsystem.setFeederVelocityVoltage(
          SmartDashboard.getNumber("FeederForwardVelocity", FeederConstants.feederForwardVelocity));
      m_barrelsubsystem.setBarrelVelocity(
          SmartDashboard.getNumber("BarrelForwardVelocity", BarrelConstants.BarrelForwardVelocity));
    }
    SmartDashboard.putNumber("xCompensation", xCompensation);
    SmartDashboard.putNumber("yCompensation", yCompensation);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("HoodPosition", 0);
    m_subsystem.setIdle(true);
    m_barrelsubsystem.stop();
    m_feedersubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
