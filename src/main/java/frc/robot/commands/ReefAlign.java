package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.function.Supplier;

public class ReefAlign extends InstantCommand {
  private Drive driveRef;
  private Supplier<List<Pose2d>> reefTagSupplier;
  private Pose2d offsetFromTag;

  private ChassisSpeeds speeds = new ChassisSpeeds();

  private PIDController xControllerRobot = Constants.xController;
  private PIDController yControllerRobot = Constants.yController;
  private PIDController thetaControllerRobot = Constants.thetaController;

  private boolean aligned = false;

  private ChassisSpeeds maximumSpeeds = new ChassisSpeeds(0.95, 1.0, 1.85);
  private Translation3d goalErrors = new Translation3d(0.015, 0.015, 0.01);

  public ReefAlign(Drive drive, Supplier<List<Pose2d>> reefTags, Pose2d offsetPose2d) {
    driveRef = drive;
    reefTagSupplier = reefTags;
    offsetFromTag = offsetPose2d;
  }

  @Override
  public void execute() {
    aligned = runRobotRelativeAlign(maximumSpeeds, goalErrors);
  }

  @Override
  public boolean isFinished() {
    if (aligned) {
      // RobotContainer.addAlert(new AlertBody(AlertMode.FULLY_ALIGNED, 0.5));
    }
    return aligned;
  }

  public boolean runRobotRelativeAlign(ChassisSpeeds maximumSpeeds, Translation3d goalErrors) {

    double maxSpeedX = Math.abs(maximumSpeeds.vxMetersPerSecond);
    double maxSpeedY = Math.abs(maximumSpeeds.vyMetersPerSecond);
    double maxSpeedTheta = Math.abs(maximumSpeeds.omegaRadiansPerSecond);

    if (reefTagSupplier.get().isEmpty()) {
      return false;
    }
    Pose2d nearestTag = driveRef.getPose().nearest(reefTagSupplier.get());
    Pose2d fieldRelativeTarget = offsetFromTag.relativeTo(nearestTag);

    Pose2d offsetPose = driveRef.getPose().relativeTo(fieldRelativeTarget);

    double xAlignSpeed =
        MathUtil.clamp(xControllerRobot.calculate(offsetPose.getX()), -maxSpeedX, maxSpeedX);
    double yAlignSpeed =
        MathUtil.clamp(yControllerRobot.calculate(offsetPose.getY()), -maxSpeedY, maxSpeedY);
    double thetaAlignSpeed =
        MathUtil.clamp(
            thetaControllerRobot.calculate(offsetPose.getRotation().getRadians()),
            -maxSpeedTheta,
            maxSpeedTheta);

    if (Math.abs(offsetPose.getX()) < goalErrors.getX()) {
      xAlignSpeed = 0.0;
    }

    if (Math.abs(offsetPose.getY()) < goalErrors.getY()) {
      yAlignSpeed = 0.0;
    }

    if (Math.abs(offsetPose.getRotation().getRadians()) < goalErrors.getZ()) {
      thetaAlignSpeed = 0.0;
    }

    speeds = new ChassisSpeeds(xAlignSpeed, yAlignSpeed, thetaAlignSpeed);

    driveRef.runVelocity(speeds);

    return xAlignSpeed == 0.0 && yAlignSpeed == 0.0 && thetaAlignSpeed == 0.0;
  }
}
