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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ClawCommands.*;
import frc.robot.commands.ClimberCommand.ClimberClawCommand;
import frc.robot.commands.ClimberCommand.ClimberSetPositionCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.commands.PivotCommands.*;
import frc.robot.commands.alignalage;
import frc.robot.commands.alignreef;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ContainerConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.claw.ClawIOSim;
import frc.robot.subsystems.claw.ClawIOTalonFX;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.pivot.PivotSubsystem;
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
  private final ClawSubsystem clawSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ClimberSubsystem climberSubsystem;

  private final RobotContainer robotContainer = this;

  // Controller
  private CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);
  private final CommandXboxController climberController = new CommandXboxController(2);

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
        clawSubsystem = new ClawSubsystem(new ClawIOTalonFX());
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOTalonFX());
        pivotSubsystem = new PivotSubsystem(new PivotIOTalonFX());
        intakeSubsystem = new IntakeSubsystem(new IntakeIOTalonFX());
        climberSubsystem = new ClimberSubsystem(new ClimberIOTalonFX());
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
        clawSubsystem = new ClawSubsystem(new ClawIOSim());
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
        pivotSubsystem = new PivotSubsystem(new PivotIOSim());
        intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
        climberSubsystem = new ClimberSubsystem(new ClimberIOTalonFX());
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
        clawSubsystem = new ClawSubsystem(new ClawIOTalonFX());
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOTalonFX());
        pivotSubsystem = new PivotSubsystem(new PivotIOTalonFX());
        intakeSubsystem = new IntakeSubsystem(new IntakeIOTalonFX());
        climberSubsystem = new ClimberSubsystem(new ClimberIOTalonFX());
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

    // Configure the button bindings

    // Default Command to set intake arm position
    intakeSubsystem.setDefaultCommand(new IntakeArmSetPositionCommand(intakeSubsystem, 0));

    NamedCommands.registerCommand("PivotLocked", new PivotSetPositionCommand(pivotSubsystem, 0));
    NamedCommands.registerCommand("L4 Command", getL4Command().withTimeout(3.5));
    NamedCommands.registerCommand("Auto Allign", new alignreef(false, drive));
    NamedCommands.registerCommand("Wait 3s", new WaitCommand(3));

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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

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

    // Main driver

    // Intake arm
    // controller.leftTrigger().whileTrue(new IntakeArmForwardCommand(intakeSubsystem, 1));
    controller
        .leftTrigger(0.2)
        .debounce(0.5, DebounceType.kFalling)
        .whileTrue(
            new IntakeArmForwardCommand(intakeSubsystem, 1.5)
                .alongWith(
                    (new ElevatorSetPositionCommand(elevatorSubsystem, 0.95))
                        .alongWith(new PivotSetPositionCommand(pivotSubsystem, 177))));
    // .andThen(new ElevatorSetPositionCommand(elevatorSubsystem, 0.83))
    // .andThen(new ClawIntakeCommand(clawSubsystem, intakeSubsystem).withTimeout(0.8))
    // .andThen(new ElevatorSetPositionCommand(elevatorSubsystem, 0.9))
    // .andThen(new PivotSetPositionCommand(pivotSubsystem, 0, 130))
    // .andThen(new ElevatorSetPositionCommand(elevatorSubsystem, 0.382))
    // .andThen(new ClawShootTimedBackCommand(clawSubsystem)));

    // L3 Command
    controller
        .a()
        .onTrue(
            new ElevatorSetPositionCommand(elevatorSubsystem, 0.879)
                .andThen(new PivotSetPositionCommand(pivotSubsystem, 43))
                .andThen(new ClawShootCommand(clawSubsystem))
                .andThen(
                    new PivotSetPositionCommand(pivotSubsystem, 0)
                        .alongWith(new ElevatorSetPositionCommand(elevatorSubsystem, 0.382))));

    // L4 Command
    controller
        .b()
        .onTrue(
            new ElevatorSetPositionCommand(elevatorSubsystem, 1.47)
                .andThen(new PivotSetPositionCommand(pivotSubsystem, 35))
                .andThen(new ClawShootCommand(clawSubsystem))
                .andThen(new PivotSetPositionCommand(pivotSubsystem, 0))
                .andThen(new ElevatorSetPositionCommand(elevatorSubsystem, 0.382)));

    //  Alage Level1 Intake
    controller
        .x()
        .whileTrue(
            new ElevatorSetPositionCommand(elevatorSubsystem, 1.38)
                .andThen(new PivotSetPositionCommand(pivotSubsystem, 120.8))
                .andThen(new ClawAlageInCommand(clawSubsystem)));

    //  Alage Level2 Intake
    controller
        .y()
        .whileTrue(
            new ElevatorSetPositionCommand(elevatorSubsystem, 1.604)
                .andThen(new PivotSetPositionCommand(pivotSubsystem, 112.8))
                .andThen(new ClawAlageInCommand(clawSubsystem)));

    // Alage Shoot
    controller
        .povLeft()
        .whileTrue(
            new ElevatorSetPositionCommand(elevatorSubsystem, 1.615)
                .andThen(
                    new PivotSetPositionCommand(pivotSubsystem, 21.5)
                        .andThen(new ClawAlageShootCommand(clawSubsystem))));

    // L2 Command
    controller
        .povRight()
        .onTrue(
            new ElevatorSetPositionCommand(elevatorSubsystem, 0.352)
                .alongWith(new PivotSetPositionCommand(pivotSubsystem, 27))
                .andThen(new ClawShootCommand(clawSubsystem)));

    controller.rightTrigger().whileTrue(new IntakeRollerIndexerOutCommand(intakeSubsystem));

    // Sub Driver

    // controller2.x().whileTrue(new IntakeRollerIndexerOutCommand(intakeSubsystem));
    controller2.a().whileTrue(new IntakeArmReverseCommand(intakeSubsystem, 2));

    controller2.b().whileTrue(new ElevatorSetPositionCommand(elevatorSubsystem, 0.382));
    controller2.povLeft().whileTrue(new PivotSetPositionCommand(pivotSubsystem, 0));

    // Auto Allignment
    controller.leftBumper().whileTrue(new alignreef(false, drive));

    controller.rightBumper().whileTrue(new alignreef(true, drive));

    controller.povDown().whileTrue(new alignalage(drive));

    // Station Intake

    // Climber
    controller2.povUp().whileTrue((new ClimberClawCommand(climberSubsystem)));
    controller2
        .leftTrigger()
        .onTrue(
            new ClimberSetPositionCommand(climberSubsystem, -420)
                .alongWith(new PivotSetPositionCommand(pivotSubsystem, 100)));

    controller2
        .rightTrigger()
        .onTrue(
            new ClimberSetPositionCommand(climberSubsystem, 340)
                .alongWith(new PivotSetPositionCommand(pivotSubsystem, 100)));

    // controller2.povDown().whileTrue(new IntakeArmSetPositionCommand(intakeSubsystem, 0));

    // controller2
    //     .leftTrigger()
    //     .whileTrue(
    //         new ClimberPivotDefaultCommand(climberSubsystem)
    //             .alongWith(new PivotSetPositionCommand(pivotSubsystem, 100)));
    // controller2.rightTrigger().whileTrue(new ClimberPivotUpCommand(climberSubsystem));
    controller2.rightBumper().whileTrue(new ElevatorSetPositionCommand(elevatorSubsystem, 1));

    // Claw Intake Command
    controller2
        .y()
        .whileTrue(
            new ElevatorSetPositionCommand(elevatorSubsystem, 0.90)
                .alongWith(new PivotSetPositionCommand(pivotSubsystem, 177))
                .andThen(
                    new ElevatorSetPositionCommand(elevatorSubsystem, 0.83)
                        .alongWith(
                            new ClawIntakeCommand(clawSubsystem, intakeSubsystem).withTimeout(0.8)))
                .andThen(new ElevatorSetPositionCommand(elevatorSubsystem, 0.9))
                .andThen(new PivotSetPositionCommand(pivotSubsystem, 0, 130))
                .andThen(new ElevatorSetPositionCommand(elevatorSubsystem, 0.382))
                .andThen(new ClawShootTimedBackCommand(clawSubsystem)));

    // controller2
    //     .y()
    //     .whileTrue(
    //         new ElevatorSetPositionCommand(elevatorSubsystem, 0.90)
    //             .alongWith(new PivotSetPositionCommand(pivotSubsystem, 177))
    //             .andThen(new WaitUntilCommand(() -> intakeSubsystem.getCanRange()))
    //             .andThen(
    //                 new ElevatorSetPositionCommand(elevatorSubsystem, 0.83)
    //                     .alongWith(new ClawIntakeCommand(clawSubsystem, intakeSubsystem)))
    //             .andThen(new ElevatorSetPositionCommand(elevatorSubsystem, 0.9))
    //             .andThen(new PivotSetPositionCommand(pivotSubsystem, 0, 130))
    //             .andThen(new ElevatorSetPositionCommand(elevatorSubsystem, 0.382))
    //             .andThen(new ClawShootTimedBackCommand(clawSubsystem)));

    controller2.leftBumper().whileTrue(new ElevatorResetPositionCommand(elevatorSubsystem));

    // Auto Test

    final DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    SmartDashboard.putData("Pathfind to Closest Reef", findToClosestReefCommand(alliance));

    SmartDashboard.putData("FirstL4", findToClosestReefCommand(alliance).andThen(getL4Command()));
  }

  public Command getRightAutoCycleCommand(Pose2d nearReafPoint, boolean isRightReef) {
    nearReafPoint = FieldConstants.inversePose2dUsingAlliance(nearReafPoint, Alliance.Blue);
    return AutoBuilder.pathfindToPose(
            AutoConstants.rightPrepareToIntakePoint,
            new PathConstraints(3, 6, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0.5)
        .withTimeout(2.0)
        .andThen(
            AutoBuilder.pathfindToPose(
                    AutoConstants.rightStation,
                    new PathConstraints(
                        0.8, 2, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                    0)
                .withTimeout(1.5)
                .alongWith(new IntakeArmForwardCommand(intakeSubsystem, 1).withTimeout(3)))
        .andThen(
            AutoBuilder.pathfindToPose(
                    nearReafPoint,
                    new PathConstraints(
                        3, 6, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                    0.5)
                .withTimeout(2.0))
        .andThen(new alignreef(isRightReef, drive).withTimeout(1.0))
        .andThen(getL4Command().withTimeout(1.0));
  }

  public Command getLeftAutoCycleCommand(Pose2d nearReafPoint, boolean isRightReef) {
    nearReafPoint = FieldConstants.inversePose2dUsingAlliance(nearReafPoint, Alliance.Blue);
    return AutoBuilder.pathfindToPose(
            AutoConstants.leftPrepareToIntakePoint,
            new PathConstraints(2, 4, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0.5)
        .withTimeout(5)
        .andThen(
            AutoBuilder.pathfindToPose(
                    AutoConstants.leftStation,
                    new PathConstraints(
                        0.8, 2, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                    0)
                .withTimeout(5)
                .alongWith(new IntakeArmForwardCommand(intakeSubsystem, 1).withTimeout(3)))
        .andThen(
            AutoBuilder.pathfindToPose(
                    nearReafPoint,
                    new PathConstraints(
                        2, 4, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                    0.5)
                .withTimeout(5))
        .andThen(new alignreef(isRightReef, drive).withTimeout(2))
        .andThen(getL4Command().withTimeout(1));
  }

  public Command getL4Command() {
    return new ElevatorSetPositionCommand(elevatorSubsystem, 1.47)
        .andThen(new PivotSetPositionCommand(pivotSubsystem, 35))
        .andThen(new ClawShootCommand(clawSubsystem))
        .andThen(new PivotSetPositionCommand(pivotSubsystem, 0))
        .andThen(new ElevatorSetPositionCommand(elevatorSubsystem, 0.382));
  }

  public Command findToClosestReefCommand(Alliance alliance) {
    Pose2d currentPose = drive.getPose();

    Pose2d closestPose =
        FieldConstants.getClosestReefPose(currentPose.getTranslation(), 1, alliance);

    return AutoBuilder.pathfindToPose(
        closestPose,
        new PathConstraints(1, 1.5, Units.degreesToRadians(360), Units.degreesToRadians(540)),
        0.0);
  }

  public Command getPivotStartCommand() {
    return new PivotSetPositionCommand(pivotSubsystem, 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("test 1m")
    //     .andThen(
    //         getRightAutoCycleCommand(
    //             new Pose2d(3.7, 2.5, new Rotation2d(Math.toRadians(-120))), true))
    //     .andThen(
    //         getRightAutoCycleCommand(
    //             new Pose2d(3.7, 2.5, new Rotation2d(Math.toRadians(-120))), false));

    return new PathPlannerAuto("L4(3)");
    // .andThen(
    //     getLeftAutoCycleCommand(
    //         new Pose2d(3.7, 5.5, new Rotation2d(Math.toRadians(120))), true))
    // .andThen(
    //     getLeftAutoCycleCommand(
    //         new Pose2d(3.7, 5.5, new Rotation2d(Math.toRadians(120))), false));
  }
}
