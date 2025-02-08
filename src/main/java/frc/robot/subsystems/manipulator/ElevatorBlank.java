package frc.robot.subsystems.manipulator;

public class ElevatorBlank implements ElevatorInterface {

  // returns the state of the limit switch
  public boolean getLimit() {
    return false;
  }

  // moves the elevator to a preset position specified by the Position parameter (created in the
  // enum)
  public void setPosition(ElevatorPosition position) {}

  public void setSpeed(double joystickInput) {}

  // moves the elevator a certain speed according to the double parameter
  public void move() {}

  public double getHeight() {
    return 0;
  }
}
