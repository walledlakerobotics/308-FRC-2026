// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Field;
import frc.robot.utils.MatchTimer;
import frc.robot.utils.MatchTimer.HubState;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  // The robot's subsystems
  private final Drivetrain m_robotDrive = new Drivetrain();

  private final Feeder m_feeder = new Feeder();
  private final Indexer m_indexer = new Indexer();
  private final Shooter m_shooter =
      new Shooter(m_robotDrive::getPose, m_robotDrive::getChassisSpeeds);
  private final Intake m_intake = new Intake();
  // private final Hood m_hood = new Hood(m_robotDrive::getPose, m_robotDrive::getChassisSpeeds);
  private final Extender m_extender = new Extender();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_coDriverController =
      new CommandXboxController(OIConstants.kCoDriverControllerPort);

  @NotLogged private final SendableChooser<Command> m_autoChooser;

  // Orchestra m_orchestra = new Orchestra();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    NamedCommands.registerCommand(
        "Shoot",
        Commands.waitUntil(m_shooter::isReady)
            .andThen(m_feeder.feed().alongWith(m_indexer.feed()))
            .alongWith(
                m_shooter.startEnd(
                    () -> m_shooter.setVelocity(RotationsPerSecond.of(65.0)), m_shooter::coast))
            .withTimeout(5.0));

    NamedCommands.registerCommand("Agitate", m_extender.agitate());

    m_autoChooser = AutoBuilder.buildAutoChooser();

    autoTab.add("Auto", m_autoChooser);

    new EventTrigger("Intake").whileTrue(m_intake.intake());

    // m_orchestra.loadMusic("eia.chrp");

    // m_extender.addOrchestra(m_orchestra);
    // m_intake.addOrchestra(m_orchestra);
    // m_feeder.addOrchestra(m_orchestra);
    // m_shooter.addOrchestra(m_orchestra);
    // m_indexer.addOrchestra(m_orchestra);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    DoubleEntry speedEntry =
        NetworkTableInstance.getDefault()
            .getTable("Tuning")
            .getDoubleTopic("Shooter Speed")
            .getEntry(0.0);
    speedEntry.set(0.0);

    // Configure drive control
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        m_robotDrive.drive(
            m_driverController::getLeftY,
            m_driverController::getLeftX,
            m_driverController::getRightX,
            true));

    // m_driverController.rightBumper().whileTrue(m_robotDrive.setX());

    m_driverController.leftBumper().onTrue(m_robotDrive.resetFieldRelative());

    m_driverController
        .rightBumper()
        .whileTrue(
            m_robotDrive.faceTowards(
                () -> Field.Landmark.Hub.getTranslation(Field.getAlliance()).toTranslation2d(),
                m_driverController::getLeftY,
                m_driverController::getLeftX,
                true));

    m_coDriverController
        .rightBumper()
        .whileTrue(
            m_shooter.startEnd(
                () -> m_shooter.setVelocity(RotationsPerSecond.of(speedEntry.get())),
                m_shooter::coast))
        .and(m_shooter::isReady)
        .whileTrue(m_feeder.feed().alongWith(m_indexer.feed()));

    // m_coDriverController
    //     .rightBumper()
    //     .whileTrue(
    //         Commands.waitUntil(m_shooter::isReady)
    //             .andThen(m_feeder.feed().alongWith(m_indexer.feed()))
    //             .alongWith(
    //                 m_shooter.startEnd(
    //                     () -> m_shooter.setVelocity(RotationsPerSecond.of(73.0)),
    //                     m_shooter::coast)));

    m_coDriverController
        .rightTrigger()
        .whileTrue(
            Commands.waitUntil(m_shooter::isReady)
                .andThen(m_feeder.feed().alongWith(m_indexer.feed()))
                .alongWith(
                    m_shooter.startEnd(
                        () -> m_shooter.setVelocity(RotationsPerSecond.of(100.0)),
                        m_shooter::coast)));

    m_coDriverController.y().whileTrue(m_indexer.reverse());

    m_coDriverController.leftBumper().whileTrue(m_intake.intake());
    m_coDriverController.leftTrigger().whileTrue(m_intake.reverse());

    // m_coDriverController.x().onTrue(m_hood.runOnce(() -> m_hood.setAngle(Degrees.of(7))));
    // m_coDriverController.a().onTrue(m_hood.runOnce(() -> m_hood.setAngle(Degrees.of(25.0))));
    // m_coDriverController.b().onTrue(m_hood.runOnce(() -> m_hood.setAngle(Degrees.of(35.0))));

    // m_coDriverController
    //     .povUp()
    //     .toggleOnTrue(
    //         m_extender
    //             .runOnce(() -> m_extender.setPosition(ExtenderConstants.kExtendedPosition))
    //             .finallyDo(() -> m_extender.setPosition(0.0)));

    m_coDriverController.povDown().whileTrue(m_extender.run());
    m_coDriverController.povUp().whileTrue(m_extender.runDown());

    m_coDriverController.povLeft().whileTrue(m_extender.agitate());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  @Logged(name = "Hub Switch Time")
  public double getHubSwitchTime() {
    HubState state =
        MatchTimer.getInstance().getMatchPeriod().getHubState().orElse(HubState.Active);

    return MatchTimer.getInstance()
        .getTimeUntilHubState(state == HubState.Active ? HubState.Inactive : HubState.Active);
  }
}
