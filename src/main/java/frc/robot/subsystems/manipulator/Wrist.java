package frc.robot.subsystems.manipulator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class Wrist implements WristInterface {

  // encoder - TBD gear ratio - TBD

  // ALL CAN IDs HIGHER THAN 12

  private double targetAngle = 0.0;
  private double targetSpeed = 0.0;
  private ControlType controlType;

  private SparkBase wristMotor;
  private DigitalInput wristLimitSwitch; // triggered = true
  private RelativeEncoder wristEncoder;
  private AbsoluteEncoder wristAbsoluteEncoder;
  private SparkClosedLoopController PIDController = wristMotor.getClosedLoopController();

  private final double wristP = 0.5;
  private final double wristI = 0.0;
  private final double wristD = 0.0;
  private final double wristFF = 0.0;
  private final double gearRatio = 1 / 12.0;
  private final double wristEncoderPositionFactor = 360 * gearRatio; // Rotor Rotations -> Degrees
  private final double wristEncoderVelocityFactor = wristEncoderPositionFactor / 60;
  private final int currentLimit = 40;
  private final double wristAngleMax = 180.0;
  private final int wristMotorCANID = 11;
  // private final int wristLimitSwitchID = 271817181;
  private final double maxWristSpeed = 5.0; // degrees per second

  public Wrist() {
    wristMotor = new SparkMax(wristMotorCANID, MotorType.kBrushless);
    // wristLimitSwitch = new DigitalInput(wristLimitSwitchID);
    wristEncoder = wristMotor.getEncoder();
    wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder();

    var driveConfig =
        new SparkFlexConfig(); // TODO: invert the motor if needed to make positive motor speed
    // increase the angle(go down) and negative motor speed decrease the
    // angle (go up)
    driveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    driveConfig.inverted(true);
    driveConfig
        .encoder
        .positionConversionFactor(wristEncoderPositionFactor)
        .velocityConversionFactor(wristEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .absoluteEncoder
        .inverted(true) // TODO check this
        .positionConversionFactor(360)
        .velocityConversionFactor(6)
        .averageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            wristP, wristI,
            wristD, wristFF);
    driveConfig
        .limitSwitch
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(false); // TODO Enable when limit switch is added
    driveConfig
        .softLimit
        .forwardSoftLimit(180) // TODO update max height in meters
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true);
    driveConfig
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
        wristMotor,
        5,
        () ->
            wristMotor.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(wristMotor, 5, () -> wristEncoder.setPosition(wristAbsoluteEncoder.getPosition()));
  }

  public void resetEncoder(double angle) {
    wristEncoder.setPosition(angle);
  }

  public double getPosition() {
    return wristEncoder.getPosition();
  }

  public boolean getLimitSwitch() {
    return wristLimitSwitch.get();
  }

  public void setWristSpeed(double joystickInput) {
    targetSpeed = joystickInput * maxWristSpeed;
    controlType = ControlType.kVelocity;
  }

  public void setWristAngle(WristAngle angle) {
    targetAngle = angle.angle;
    controlType = ControlType.kPosition;
  }

  public void updateWrist() {
    /*
    if(wristLimitSwitch.get()){
        resetEncoder(0.0);
    }
    */
    double PIDTarget = targetSpeed;
    if (controlType == ControlType.kPosition) PIDTarget = targetAngle;
    PIDController.setReference(PIDTarget, controlType);
  }
}
