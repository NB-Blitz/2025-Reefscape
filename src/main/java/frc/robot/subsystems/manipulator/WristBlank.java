package frc.robot.subsystems.manipulator;

public class WristBlank implements WristInterface {

  public void resetEncoder(double angle) {}

  public double getPosition() {
    return 0;
  }

  public boolean getLimitSwitch() {
    return false;
  }

  public void setWristSpeed(double joystickInput) {}

  public void setWristAngle(WristAngle angle) {}

  public void updateWrist() {}
}
