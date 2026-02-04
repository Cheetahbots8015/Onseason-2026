// ExampleSubsystem - Subsystem to control a single TalonFX motor for a example

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  
  public ClimberIO getIO() {
    return io;
  }

  public void setClimberVoltage(double volts) {
    io.setClimberVoltage(volts);
  }


  public ClimberIOInputs getInput() {
    return inputs;
  }

  public void setClimberPosition(double degree) {
    io.setClimberPosition(degree);
  }


  public void resetClimberPosition() {
    io.resetClimberPosition();
  }

  public void stopClimber() {
    io.stopClimber();
  }
}
