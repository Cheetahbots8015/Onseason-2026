// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.drive.Drive;

public class alignreef extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private Drive m_drive;
  private double tagID = -1;
  private boolean aligny;

  public alignreef(boolean isRightScore, Drive drive) {
    xController =
        new PIDController(LimelightConstants.X_REEF_ALIGNMENT_P, 0.0, 0); // Vertical movement
    yController =
        new PIDController(LimelightConstants.Y_REEF_ALIGNMENT_P, 0.0, 0); // Horitontal movement
    rotController = new PIDController(LimelightConstants.ROT_REEF_ALIGNMENT_P, 0, 0); // Rotation
    this.isRightScore = isRightScore;
    this.m_drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.aligny = true;
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(LimelightConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(LimelightConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(LimelightConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(LimelightConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? 0.15 : -0.21);
    yController.setTolerance(LimelightConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    if (LimelightHelpers.getTV("limelight-left") && !LimelightHelpers.getTV("limelight-right")) {
      tagID = LimelightHelpers.getFiducialID("limelight-left");
    } else if (!LimelightHelpers.getTV("limelight-left")
        && LimelightHelpers.getTV("limelight-right")) {
      tagID = LimelightHelpers.getFiducialID("limelight-right");
    } else if (LimelightHelpers.getTV("limelight-left")
        && LimelightHelpers.getTV("limelight-right")) {
      if (LimelightHelpers.getTA("limelight-left") > LimelightHelpers.getTA("limelight-right")) {
        tagID = LimelightHelpers.getFiducialID("limelight-left");
      } else {
        tagID = LimelightHelpers.getFiducialID("limelight-right");
      }
    }
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("aligny", aligny);
    if (LimelightHelpers.getTV("limelight-left")
        && LimelightHelpers.getFiducialID("limelight-left") == tagID) {

      this.dontSeeTagTimer.reset();
      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
      double xSpeed = -xController.calculate(postions[2]);
      double ySpeed = yController.calculate(postions[0]);
      double rotValue = rotController.calculate(postions[4]);
      if (!yController.atSetpoint() && aligny) {
        ChassisSpeeds drivSpeeds = new ChassisSpeeds(0, ySpeed, rotValue);
        m_drive.runVelocity(drivSpeeds);
      } else {
        aligny = false;
        ChassisSpeeds drivSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotValue);
        m_drive.runVelocity(drivSpeeds);
      }
      if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
        stopTimer.reset();
      }

    } else if (LimelightHelpers.getTV("limelight-right")
        && LimelightHelpers.getFiducialID("limelight-right") == tagID) {

      this.dontSeeTagTimer.reset();
      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-right");
      double xSpeed = -xController.calculate(postions[2]);
      double ySpeed = yController.calculate(postions[0]);
      double rotValue = rotController.calculate(postions[4]);
      if (!yController.atSetpoint() && aligny) {
        ChassisSpeeds drivSpeeds = new ChassisSpeeds(0, ySpeed, rotValue);
        m_drive.runVelocity(drivSpeeds);
      } else {
        aligny = false;
        ChassisSpeeds drivSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotValue);
        m_drive.runVelocity(drivSpeeds);
      }
      if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
        stopTimer.reset();
      }

    } else {
      ChassisSpeeds drivSpeeds = new ChassisSpeeds(-0.5, 0, 0);
      m_drive.runVelocity(drivSpeeds);
    }
    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds drivSpeeds = new ChassisSpeeds(0, 0, 0);
    m_drive.runVelocity(drivSpeeds);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag
    // in the camera
    return this.dontSeeTagTimer.hasElapsed(LimelightConstants.DONT_SEE_TAG_WAIT_TIME)
        || stopTimer.hasElapsed(LimelightConstants.POSE_VALIDATION_TIME);
  }
}
