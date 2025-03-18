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

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Vision vision;
  private final Drive drive;
  private final Manipulator manipulator;
  private final Climber climber;

  // Constant to switch between the practice SDS base and the competition Flex base
  private final boolean useSecondController = true;

  // Controllers
  private final CommandJoystick joystick = new CommandJoystick(0);
  private final CommandXboxController xBoxController;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (useSecondController) {
      xBoxController = new CommandXboxController(1);
    } else {
      xBoxController = null;
    }
    manipulator = new Manipulator();
    climber = new Climber();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        if (DriveConstants.compRobot) {
          drive =
              new Drive(
                  new GyroIONavX(),
                  new ModuleIOSparkFlex(0),
                  new ModuleIOSparkFlex(1),
                  new ModuleIOSparkFlex(2),
                  new ModuleIOSparkFlex(3));
        } else {
          drive =
              new Drive(
                  new GyroIONavX(),
                  new ModuleIOSparkMax(0),
                  new ModuleIOSparkMax(1),
                  new ModuleIOSparkMax(2),
                  new ModuleIOSparkMax(3));
        }
        vision = null;
        // new Vision(
        //     drive::addVisionMeasurement,
        //     new VisionIOLimelight(camera0Name, drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});

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
    configureButtonBindings();
    NamedCommands.registerCommand("resetGyro", Commands.runOnce(() -> drive.resetGyro(), drive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default Commands, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -1 * joystick.getY(),
            () -> -1 * joystick.getX(),
            () -> -1 * joystick.getZ(),
            () -> 0.5 * (1 + -joystick.getRawAxis(3))));

    // Lock to 0° when A button is held
    joystick
        .button(11)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -joystick.getY(), () -> -joystick.getX(), () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    joystick.button(4).onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    joystick
        .button(7)
        .onTrue(Commands.runOnce(() -> drive.resetGyro(), drive).ignoringDisable(true));

    // Auto aim command example
    @SuppressWarnings("resource")
    PIDController aimController = new PIDController(1.0, 0.0, 0.0);
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    joystick
        .button(6)
        .whileTrue(
            Commands.startRun(
                () -> {
                  aimController.reset();
                },
                () -> {
                  drive.runVelocity(
                      new ChassisSpeeds(
                          0.0, aimController.calculate(vision.getTargetX(0).getRadians()), 0.0));
                },
                drive));
    joystick.button(8).onTrue(Commands.runOnce(() -> climber.deploy(), climber));
    joystick.button(9).onTrue(Commands.runOnce(() -> climber.retract(), climber));
    joystick.button(8).onFalse(Commands.runOnce(() -> climber.stop(), climber));
    joystick.button(9).onFalse(Commands.runOnce(() -> climber.stop(), climber));
    if (useSecondController) {
      manipulator.setDefaultCommand(
          ManipulatorCommands.joystickManipulator(
              manipulator,
              () -> -xBoxController.getLeftTriggerAxis(),
              () -> xBoxController.getRightTriggerAxis(),
              () -> -xBoxController.getLeftY(),
              () -> -xBoxController.getRightY()));
      xBoxController
          .povUp()
          .onTrue(Commands.runOnce(() -> manipulator.incrementLevel(), manipulator));
      xBoxController
          .povDown()
          .onTrue(Commands.runOnce(() -> manipulator.decrementLevel(), manipulator));
      xBoxController.leftBumper().onTrue(Commands.runOnce(() -> manipulator.intake(), manipulator));
      xBoxController.rightBumper().onTrue(Commands.runOnce(() -> manipulator.expel(), manipulator));
      xBoxController
          .leftBumper()
          .onFalse(Commands.runOnce(() -> manipulator.stopHand(), manipulator));
      xBoxController
          .rightBumper()
          .onFalse(Commands.runOnce(() -> manipulator.stopHand(), manipulator));
      xBoxController
          .leftStick()
          .onTrue(
              Commands.run(
                  () -> {
                    if (xBoxController.rightStick().getAsBoolean()) {
                      manipulator.emergencyStop();
                    }
                  },
                  manipulator));
    }
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
