package frc.robot.subsystems.manipulator;

public interface WristInterface {

  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public enum WristAngle {
    coralBottom(-1),
    coralMiddle(-1),
    coralTop(-1),
    coralIntake(-1),
    algaeInReef(-1),
    algaeBarge(-1),
    algaeProcessor(-1),
    algaeIntake(-1),
    bottom(-1),
    top(-1);

    public final double angle;

    WristAngle(double angle) {
      this.angle = angle;
    }
  }

  public void resetEncoder(double angle);

  public double getPosition();

  public boolean getLimitSwitch();

  public void setWristSpeed(double joystickInput);

  public void setWristAngle(WristAngle angle);

  public void updateWrist();
}
