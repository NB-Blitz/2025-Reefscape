package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.subsystems.common.Joint;

public class Shoulder extends Joint {

  private static final double jointP = 0.0065;
  private static final double jointI = 0.0;
  private static final double jointD = 0.0;
  private static final double jointFF = 0.0;
  private static final double gearRatio = 1 / (64 * 3.6);
  private static final int jointMotorCANID = 11;
  private static final double maxJointSpeed = 180.0; // degrees per second

  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public enum ShoulderAngle {
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
        135.0,
        0.0,
        5.0,
        new SparkFlex(jointMotorCANID, MotorType.kBrushless),
        new SparkFlexConfig());
  }

  @Override
  public void setJointAngle(int enumIndex) {
    super.targetAngle = ShoulderAngle.values()[enumIndex].angle + angleOffset;
    super.controlType = ControlType.kPosition;
  }

  @Override
  public void updateJoint()
  {
    super.updateJoint();
    Logger.recordOutput("Manipulator/Shoulder/Position", getPosition());
    Logger.recordOutput("Manipulator/Shoulder/Current", jointMotor.getOutputCurrent());
  }
}
