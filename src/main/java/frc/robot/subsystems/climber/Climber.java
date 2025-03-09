package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.common.Joint;
import frc.robot.subsystems.common.JointBlank;

public class Climber extends SubsystemBase {
  private final Joint climberJoint;
  private final double p = 0.0;
  private final double i = 0.0;
  private final double d = 0.0;
  private final double ff = 0.0;
  private final double gearRatio = 1 / 108.0;
  private final double maxSpeed = 90;
  private final boolean invert = false;
  private final int canID = 15;

  public Climber() {

    climberJoint = new JointBlank();
    // new Joint(
    //     p,
    //     i,
    //     d,
    //     ff,
    //     gearRatio,
    //     maxSpeed,
    //     invert,
    //     new SparkFlex(canID, MotorType.kBrushless),
    //     new SparkFlexConfig());

    // this is normal converstion with Jackson
    // Alex: my house is so big... it massive
    // Jackson: You know what is massive
    // Everet: The ninja LOOWWW TAPER
    // Jackson and Everet: FADE!
    // Jackson: look at the tv... EMILIO look what it says
    // Emilio: Yeah...
    // btw for someone who is reading this... be warned... you... are... breathing...
    // you have now activated nonouto breathing mode.
    // and you are the key to carrying on the Ninja Low Taper Fade Meme

    // in the night
    // in a room without light
    // there shines a fight
    // a poetic knight
    // destroying the blight
    // away from this gleaming world.
    // destined to save the world.
    // yet the blight struck

  }

  public void deploy() {
    climberJoint.setJointSpeed(0.2);
  }

  public void retract() {
    climberJoint.setJointSpeed(-0.2);
  }

  public void stop() {
    climberJoint.setJointSpeed(0.0);
  }

  public void periodic() {
    climberJoint.updateJoint();
  }
}
