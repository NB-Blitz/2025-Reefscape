package frc.robot.subsystems.common;

public class JointBlank extends Joint {

  public JointBlank() {
    super();
  }

  public void resetEncoder(double angle) {}

  public double getPosition() {
    return 0;
  }

  public void setJointSpeed(double joystickInput) {}

  public void updateJoint() {}
}
