package frc.robot.subsystems.manipulator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;

public class Joint {

  // encoder - TBD gear ratio - TBD

  // ALL CAN IDs HIGHER THAN 12

  protected double targetAngle = 0.0;
  private double targetSpeed = 0.0;
  protected ControlType controlType = ControlType.kDutyCycle;

  private SparkBase jointMotor;
  private DigitalInput jointLimitSwitch; // triggered = true
  private RelativeEncoder jointEncoder;
  private AbsoluteEncoder jointAbsoluteEncoder;
  private SparkClosedLoopController jointController;
  private final int currentLimit = 40;
  private final double maxJointSpeed;

  public Joint(
      double P,
      double I,
      double D,
      double FF,
      double gearRatio,
      double maxSpeed,
      double absGearRatio,
      boolean absInvert,
      boolean invert,
      SparkBase motorRef,
      SparkBaseConfig config) {
    jointMotor = motorRef;
    jointController = jointMotor.getClosedLoopController();
    this.maxJointSpeed = maxSpeed;
    // wristLimitSwitch = new DigitalInput(wristLimitSwitchID);
    jointEncoder = jointMotor.getEncoder();
    jointAbsoluteEncoder = jointMotor.getAbsoluteEncoder();

    var jointConfig = config;
    // TODO: invert the motor if needed to make positive motor speed
    // increase the angle(go down) and negative motor speed decrease the
    // angle (go up)
    jointConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    jointConfig.inverted(invert);
    jointConfig
        .encoder
        .positionConversionFactor(gearRatio * 360)
        .velocityConversionFactor(gearRatio * 6)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    jointConfig
        .absoluteEncoder
        .inverted(absInvert) // TODO check this
        .positionConversionFactor(absGearRatio * 360)
        .velocityConversionFactor(absGearRatio * 6)
        .averageDepth(2);
    jointConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            P, I,
            D, FF);
    jointConfig
        .limitSwitch
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(false); // TODO Enable when limit switch is added
    jointConfig
        .softLimit
        .forwardSoftLimit(180) // TODO update max height in meters
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true);
    jointConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(10)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        jointMotor,
        5,
        () ->
            jointMotor.configure(
                jointConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(jointMotor, 5, () -> jointEncoder.setPosition(jointAbsoluteEncoder.getPosition()));
  }

  public Joint() {
    maxJointSpeed = 0;
  }

  public void resetEncoder(double angle) {
    jointEncoder.setPosition(angle);
  }

  public double getPosition() {
    return jointEncoder.getPosition();
  }

  // public boolean getLimitSwitch() {
  //   return jointLimitSwitch.get();
  // }

  public void setJointSpeed(double joystickInput) {
    targetSpeed = joystickInput * maxJointSpeed;
    controlType = ControlType.kVelocity;
  }

  public void setJointAngle(int enumIndex) {}

  public void updateJoint() {
    /*
    if(wristLimitSwitch.get()){
        resetEncoder(0.0);
    }
    */
    double PIDTarget = targetSpeed;
    if (controlType == ControlType.kPosition) PIDTarget = targetAngle;
    jointController.setReference(PIDTarget, controlType);
  }
}
