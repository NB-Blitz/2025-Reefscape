package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.common.Joint;

public class Wrist extends Joint {

  private static final double jointP = 0.5;
  private static final double jointI = 0.0;
  private static final double jointD = 0.0;
  private static final double jointFF = 0.0;
  private static final double gearRatio = 1 / 12.0;
  private final double jointEncoderPositionFactor = 360 * gearRatio; // Rotor Rotations -> Degrees
  private final double jointEncoderVelocityFactor = jointEncoderPositionFactor / 60;
  private static final int jointMotorCANID = 11;
  // private final int wristLimitSwitchID = 271817181;
  private static final double maxJointSpeed = 5.0; // degrees per second

  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public enum WristAngle {
    coralBottom(90),
    coralMiddle(45),
    coralTop(30),
    coralIntake(90),
    algaeInReef(90),
    algaeBarge(135),
    algaeProcessor(45),
    algaeIntake(45),
    bottom(0),
    top(180);

    public final double angle;

    WristAngle(double angle) {
      this.angle = angle;
    }
  }

  public Wrist() {
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
        new SparkMax(jointMotorCANID, MotorType.kBrushless),
        new SparkMaxConfig());
  }

  public void setJointAngle(int enumIndex) {
    super.targetAngle = WristAngle.values()[enumIndex].angle;
    super.controlType = ControlType.kPosition;
  }
}
