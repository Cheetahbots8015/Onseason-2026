// ExampleSubsystem - Subsystem to control a single TalonFX motor for a example

package frc.robot.subsystems.barrel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.barrel.BarrelIO.BarrelIOInputs;
import org.littletonrobotics.junction.Logger;

public class BarrelSubsystem extends SubsystemBase {
  private final BarrelIO io;
  private final BarrelIOInputsAutoLogged inputs = new BarrelIOInputsAutoLogged();

  public BarrelSubsystem(BarrelIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Barrel", inputs);
  }

  public BarrelIO getIO() {
    return io;
  }

  public BarrelIOInputs getInput() {
    return inputs;
  }

  public void setBarrelVoltage(double volts){
    io.setBarrelVoltage(volts);
  }

  public void stop(){
    io.stop();
  }
}
