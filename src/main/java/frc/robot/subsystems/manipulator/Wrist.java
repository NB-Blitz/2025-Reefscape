package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.subsystems.common.Joint;
import org.littletonrobotics.junction.Logger;

public class Wrist extends Joint {

  private static final double jointP = 0.001;
  private static final double jointI = 0.0;
  private static final double jointD = 0.0;
  private static final double jointFF = 0.0;
  private static final double gearRatio = 1 / 90.0;
  private static final int jointMotorCANID = 12;
  private static final double maxJointSpeed = 180.0; // degrees per second

  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public enum WristAngle {
    coralL1(145),
    coralL2(118),
    coralL3(94),
    coralL4(130),
    coralIntake(152),
    algaeInReefL1(118),
    algaeInReefL2(118),
    algaeBarge(55),
    algaeProcessor(152),
    bottom(155);

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
        false,
        155.0,
        0,
        5.0,
        new SparkFlex(jointMotorCANID, MotorType.kBrushless),
        new SparkFlexConfig());
  }

  @Override
  public void setJointAngle(int enumIndex) {
    super.targetAngle = WristAngle.values()[enumIndex].angle + angleOffset;
    super.controlType = ControlType.kPosition;
  }

  @Override
  public void updateJoint() {
    super.updateJoint();
    Logger.recordOutput("Manipulator/Wrist/Position", getPosition());
    Logger.recordOutput("Manipulator/Wrist/Current", jointMotor.getOutputCurrent());
  }
}
