package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Manipulator;
import java.util.function.DoubleSupplier;

public class ManipulatorCommands {

  public ManipulatorCommands() {}

  public static Command joystickManipulator(
      Manipulator manipulator, DoubleSupplier leftYSupplier, DoubleSupplier rightYSupplier) {
    return Commands.run(
        () -> {
          manipulator.runManipulator(leftYSupplier.getAsDouble(), rightYSupplier.getAsDouble());
        });
  }
}
