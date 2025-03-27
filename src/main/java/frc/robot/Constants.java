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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final Transform2d[] rightReef =
      new Transform2d[] {
        new Transform2d(0.6, 0.165, Rotation2d.fromDegrees(180.0)),
        new Transform2d(0.5, 0.165, Rotation2d.fromDegrees(180.0)),
        new Transform2d(0.4, 0.165, Rotation2d.fromDegrees(180.0)),
        new Transform2d(0.45, 0.16, Rotation2d.fromDegrees(180.0))
      };
  public static final Transform2d[] leftReef =
      new Transform2d[] {
        new Transform2d(0.6, -0.185, Rotation2d.fromDegrees(180.0)),
        new Transform2d(0.5, -0.185, Rotation2d.fromDegrees(180.0)),
        new Transform2d(0.4, -0.185, Rotation2d.fromDegrees(180.0)),
        new Transform2d(0.45, -0.19, Rotation2d.fromDegrees(180.0))
      };

  public static final PIDController xController = new PIDController(4, 0, 0);
  public static final PIDController yController = new PIDController(4, 0, 0);
  public static final PIDController thetaController = new PIDController(7, 0, 0);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
