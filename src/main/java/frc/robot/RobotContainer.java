// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.BarrelCommands.BarrelForwardVelocityCommand;
import frc.robot.commands.BarrelCommands.BarrelReverseVelocityCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeederCommands.FeederForwardVelocityCommand;
import frc.robot.commands.FeederCommands.FeederReverseVelocityCommand;
import frc.robot.commands.HoodCommands.setHoodPositionCommand;
import frc.robot.commands.IntakeCommands.FlyWheelSetVelocityCommand;
import frc.robot.commands.ShooterCommands.ShooterSetVelocityCommand;
import frc.robot.constants.ContainerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.barrel.BarrelIOSim;
import frc.robot.subsystems.barrel.BarrelIOTalonFX;
import frc.robot.subsystems.barrel.BarrelSubsystem;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.TurretSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final BarrelSubsystem barrel;
  public final ShooterSubsystem shooter;
  private final HoodSubsystem hood;
  private final FeederSubsystem feeder;
  private final TurretSubsystem turret;
  private final ClimberSubsystem climber;
  private final IntakeSubsystem intake;

  // Controller
  private CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (ContainerConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2() {},
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        barrel = new BarrelSubsystem(new BarrelIOTalonFX());
        shooter = new ShooterSubsystem(new ShooterIOTalonFX());
        hood = new HoodSubsystem(new HoodIOTalonFX());
        feeder = new FeederSubsystem(new FeederIOTalonFX());
        turret = new TurretSubsystem(new TurretIOTalonFX());
        climber = new ClimberSubsystem(new ClimberIOTalonFX());
        intake = new IntakeSubsystem(new IntakeIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        barrel = new BarrelSubsystem(new BarrelIOSim());
        shooter = new ShooterSubsystem(new ShooterIOSim());
        hood = new HoodSubsystem(new HoodIOSim());
        feeder = new FeederSubsystem(new FeederIOSim());
        turret = new TurretSubsystem(new TurretIOSim());
        climber = new ClimberSubsystem(new ClimberIOSim());
        intake = new IntakeSubsystem(new IntakeIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        barrel = new BarrelSubsystem(new BarrelIOTalonFX());
        shooter = new ShooterSubsystem(new ShooterIOTalonFX());
        hood = new HoodSubsystem(new HoodIOTalonFX());
        feeder = new FeederSubsystem(new FeederIOTalonFX());
        turret = new TurretSubsystem(new TurretIOTalonFX());
        climber = new ClimberSubsystem(new ClimberIOTalonFX());
        intake = new IntakeSubsystem(new IntakeIOTalonFX());
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> Math.abs(controller.getLeftY()) > 0.2 ? -controller.getLeftY() * 0.8 : 0,
            () -> Math.abs(controller.getLeftX()) > 0.2 ? -controller.getLeftX() * 0.8 : 0,
            () -> Math.abs(controller.getRightX()) > 0.2 ? -controller.getRightX() : 0));

    controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Rotation2d heading =
                          DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
                              ? new Rotation2d(Math.PI)
                              : new Rotation2d();

                      drive.setPose(new Pose2d(drive.getPose().getTranslation(), heading));
                    },
                    drive)
                .ignoringDisable(true));

    controller.leftTrigger().whileTrue(new ShooterSetVelocityCommand(shooter));

    controller
        .rightTrigger()
        .whileTrue(
            new FeederForwardVelocityCommand(feeder)
                .alongWith(new BarrelForwardVelocityCommand(barrel)));
    controller
        .rightBumper()
        .whileTrue(
            new FeederReverseVelocityCommand(feeder)
                .alongWith(new BarrelReverseVelocityCommand(barrel)));

    controller.a().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller.b().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    controller.x().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    controller.y().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putNumber("kShootingSpeed", 220);
    SmartDashboard.putNumber("kShootingVoltage", 2.0);
    SmartDashboard.putNumber("HoodPosition", 280);
    SmartDashboard.putNumber("FlyWheelVelocity", 400);

    controller.povUp().whileTrue(new FlyWheelSetVelocityCommand(intake));
    controller.povDown().onTrue(new setHoodPositionCommand(hood));

    // Units: Degrees

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
