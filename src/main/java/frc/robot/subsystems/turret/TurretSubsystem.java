package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  private boolean isRedAlliance = false;
  private boolean inNeutralZone = false;

  private double calculated_angle = 0.0;
  Translation2d target = new Translation2d(0.0, 0.0); // Placeholder for target translation

  public TurretSubsystem(TurretIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setMotorVoltage(voltage.in(Units.Volt)), null, this));
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      target =
          new Translation2d(
              (12.519177399999998 + 11.3118646) / 2, 4.0346376); // Target for Red alliance
    } else {
      target =
          new Translation2d(
              (5.229174199999999 + 4.0218614) / 2, 4.0346376); // Target for Blue alliance
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    Translation2d currentPose =
        new Translation2d(
            SmartDashboard.getNumberArray("translation", new double[] {0.0, 0.0})[0],
            SmartDashboard.getNumberArray("translation", new double[] {0.0, 0.0})[1]);

    double rotation = SmartDashboard.getNumber("rotation", 0.0);
    
    SmartDashboard.putNumberArray(
        "shooter/translation", new double[] {currentPose.getX(), currentPose.getY()});

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
          isRedAlliance = true;
      target =
          new Translation2d(
              (12.519177399999998 + 11.3118646) / 2, 4.0346376); // Target for Red alliance
      }
      else{
            isRedAlliance = false;
        target =
            new Translation2d(
                (5.229174199999999 + 4.0218614) / 2, 4.0346376); // Target for Blue alliance
      }
    if (currentPose.getX() < 11.3118646 && currentPose.getX() > 5.229174199999999) {
      inNeutralZone = true;
    } else {
      inNeutralZone = false;
    }
    if(isRedAlliance&&!inNeutralZone || !isRedAlliance&&inNeutralZone){
      calculated_angle = Math.toDegrees(Math.atan2(target.getY() - currentPose.getY(), target.getX() - currentPose.getX())-rotation+180);
    }
    else{
      calculated_angle = Math.toDegrees(Math.atan2(target.getY() - currentPose.getY(), target.getX() - currentPose.getX())-rotation);
    }    
    calculated_angle = (calculated_angle + 360) % 360; // Normalize to [0, 360)
    SmartDashboard.putNumber("calculated_angle", calculated_angle);

  }

  public void setMotorVoltage(double volts) {
    io.setMotorVoltage(volts);
  }

  /** Set turret position in degrees */
  public void setPosition(double positionDeg) {
    io.setPosition(positionDeg);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
