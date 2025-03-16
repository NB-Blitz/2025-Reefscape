package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.subsystems.common.Joint;
import org.littletonrobotics.junction.Logger;

public class Shoulder extends Joint {

  private static final double jointP = 0.02;
  private static final double jointI = 0.0;
  private static final double jointD = 0.0;
  private static final double jointFF = 0.0;
  private static final double gearRatio = 1 / (64 * 3.6);
  private static final int jointMotorCANID = 11;
  private static final double maxJointSpeed = 135.0; // degrees per second

  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public enum ShoulderAngle {
    coralL1(12),
    coralL2(45),
    coralL3(66.5),
    coralL4(150),
    coralIntake(35.5),
    algaeInReefL1(45),
    algaeInReefL2(45),
    algaeBarge(150),
    algaeProcessor(12),
    noFoul(95.5),
    bottom(0);

    public final double angle;

    ShoulderAngle(double angle) {
      this.angle = angle;
    }
  }

  public Shoulder() {
    super(
        jointP,
        jointI,
        jointD,
        jointFF,
        gearRatio,
        maxJointSpeed,
        1.0,
        true,
        true,
        150.0,
        0.0,
        7.0,
        new SparkFlex(jointMotorCANID, MotorType.kBrushless),
        new SparkFlexConfig());
  }

  @Override
  public void setJointAngle(int enumIndex) {
    super.endTargetAngle = ShoulderAngle.values()[enumIndex].angle + angleOffset;
    super.controlMode = "preset";
  }

  @Override
  public void updateJoint() {
    super.updateJoint();
    Logger.recordOutput("Manipulator/Shoulder/Angle", getPosition());
    Logger.recordOutput("Manipulator/Shoulder/Current", jointMotor.getOutputCurrent());
    Logger.recordOutput("Manipulator/Shoulder/Target Angle", targetAngle - angleOffset);
  }
}
