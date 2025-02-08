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

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class FlexDriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(23.0);
  public static final double wheelBase = Units.inchesToMeters(23.0);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, TODO Max see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0);

  // Device CAN IDs

  public static final int frontLeftDriveCanId = 1;
  public static final int backLeftDriveCanId = 3;
  public static final int frontRightDriveCanId = 5;
  public static final int backRightDriveCanId = 7;

  public static final int frontLeftTurnCanId = 2;
  public static final int backLeftTurnCanId = 4;
  public static final int frontRightTurnCanId = 6;
  public static final int backRightTurnCanId = 8;

  // Drive motor configuration
  // TODO Max Last year's code had driving motor inverted.
  public static final boolean driveInverted = true;
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  public static final double driveMotorReduction =
      8.14; // MAXSwerve with 14 pinion teeth and 22 spur teeth
  // TODO Max Check this. Last year code does not use this. This is being used by Path plannner and
  // simulation
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  // TODO Max Check these. Last year code uses Wheel Diameter * PI need to check if wheel diameter
  // is factored into calculations later
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  // TODO Max added last year's PID for SDS in comments
  public static final double driveKp = 0.01; // 0.15
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.45; // ?
  // FF 0.45
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 50;
  public static final double turnMotorReduction = 12.8;
  // TODO Max Check this. Last year code does not use this. Updated verified this is only for
  // simulation
  public static final DCMotor turnGearbox = DCMotor.getNeoVortex(1);

  // Turn encoder configuration
  // TODO Max Unused for SDS modules
  public static final boolean turnEncoderInverted = true;
  // TODO Max Last year's code divides by motor reduction.
  public static final double turnEncoderPositionFactor =
      (2 * Math.PI) / turnMotorReduction; // Rotations -> Radians
  // TODO Max if motor reduction is added above, replace 2PI with Position Factor.
  public static final double turnEncoderVelocityFactor =
      turnMotorReduction / 60.0; // (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  public static final double absoluteTurnEncoderPositionFactor = 2 * Math.PI;
  // TODO Max if motor reduction is added above, replace 2PI with Position Factor.
  public static final double absoluteTurnEncoderVelocityFactor =
      turnMotorReduction / 60.0; // (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  // TODO Max added last year's PID for SDS in comments
  public static final double turnKp = 0.45; // 0.45
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
