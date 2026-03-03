package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GameData extends SubsystemBase {

  public GameData() {}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("can_shoot", canShoot());
    SmartDashboard.putNumber("Match_Time", DriverStation.getMatchTime());
  }

  public boolean canShoot() {
    boolean canShootBool = true;
    if (DriverStation.getAlliance().isEmpty()) {
      return true;
    }
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();
    boolean is_blue = (alliance == DriverStation.Alliance.Blue);
    String gameData = DriverStation.getGameSpecificMessage();
    double matchTime = DriverStation.getMatchTime();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          // Blue case code
          if (is_blue && (matchTime <= 140 && matchTime >= 130)) {
            canShootBool = true;
          } else if (!is_blue && (matchTime <= 130 && matchTime >= 105)) {
            canShootBool = true;
          } else if (is_blue && (matchTime <= 105 && matchTime >= 55)) {
            canShootBool = true;
          } else if (!is_blue && (matchTime <= 55 && matchTime >= 30)) {
            canShootBool = true;
          }
          break;
        case 'R':
          if (!is_blue && (matchTime <= 140 && matchTime >= 130)) {
            canShootBool = false;
          } else if (is_blue && (matchTime <= 130 && matchTime >= 105)) {
            canShootBool = false;
          } else if (!is_blue && (matchTime <= 105 && matchTime >= 55)) {
            canShootBool = false;
          } else if (is_blue && (matchTime <= 55 && matchTime >= 30)) {
            canShootBool = false;
          }
          break;
          // Red case code

        default:
          // This is corrupt data
          canShootBool = true;
          break;
      }
    } else {
      // Code for no data received yet
      canShootBool = true;
    }
    return canShootBool;
  }
}
