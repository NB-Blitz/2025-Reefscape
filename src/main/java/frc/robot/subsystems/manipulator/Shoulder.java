package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Shoulder extends Joint{

    private final static double jointP = 0.5;
    private final static double jointI = 0.0;
    private final static double jointD = 0.0;
    private final static double jointFF = 0.0;
    private final static double gearRatio = 1 / 12.0;
    private final double jointEncoderPositionFactor = 360 * gearRatio; // Rotor Rotations -> Degrees
    private final double jointEncoderVelocityFactor = jointEncoderPositionFactor / 60;
    private final static int jointMotorCANID = 11;
    // private final int wristLimitSwitchID = 271817181;
    private final static double maxJointSpeed = 5.0; // degrees per second

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

    public Shoulder(){
        super(jointP, jointI, jointD, jointFF, gearRatio, maxJointSpeed, 1.0, true, true, new SparkMax(jointMotorCANID, MotorType.kBrushless), new SparkMaxConfig());
    }

    public void setJointAngle(int enumIndex) {
        super.targetAngle = ShoulderAngle.values()[enumIndex].angle;
        super.controlType = ControlType.kPosition;
    }
}
