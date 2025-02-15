package frc.robot.subsystems.manipulator;

public interface WristInterface {

  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public enum WristAngle {
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
