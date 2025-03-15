package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.subsystems.common.Joint;
import org.littletonrobotics.junction.Logger;

public class Wrist extends Joint {

  private static final double jointP = 0.02;
  private static final double jointI = 0.0;
  private static final double jointD = 0.0;
  private static final double jointFF = 0.0;
  private static final double gearRatio = 1 / 90.0;
  private static final int jointMotorCANID = 12;
  private static final double maxJointSpeed = 90.0; // degrees per second

  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public enum WristAngle {
    coralL1(142),
    coralL2(115),
    coralL3(91),
    coralL4(127),
    coralIntake(149),
    algaeInReefL1(115),
    algaeInReefL2(115),
    algaeBarge(52),
    algaeProcessor(149),
    noFoul(66),
    bottom(152);

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
        152.0,
        50,
        20.0,
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
    Logger.recordOutput("Manipulator/Wrist/Angle", getPosition());
    Logger.recordOutput("Manipulator/Wrist/Current", jointMotor.getOutputCurrent());
    Logger.recordOutput("Manipulator/Wrist/Target Angle", super.targetAngle - angleOffset);
  }
}
