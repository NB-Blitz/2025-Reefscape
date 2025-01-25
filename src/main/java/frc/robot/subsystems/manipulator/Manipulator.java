package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase{
    private final Hand robotHand = new Hand();
    //private final Wrist robotWrist = new Wrist();
    //private final Elevator robotElevator = new Elevator();

    public void periodic() {
        /**
         * Any automtic behavior we want
         * An example would be if we have a note and are in the top position
         * we could start the shooting motors.
         */


    }

    public void intakeCoralFromStation() {
        //check if algae or coral is already in hand
        //put elevator into the coral station position
        //move the wrist into the coral station position 
        //intake coral using hand
    }

    public void scoreCoralL1() {
        //check if coral is in hand
        //move elevator into the L1 position
        //move the wrist into the L1 position
        //expel the coral
    }

    public void scoreCoralL2() {
        //check if coral is in hand
        //move elevator into the L2/L3 position
        //move the wrist into the L2/L3 position
        //expel the coral
    }

    public void scoreCoralL3() {
        //check if coral is in hand
        //move elevator into the L2/L3 position
        //move the wrist into the L2/L3 position
        //expel the coral 
    }

    public void scoreCoralL4() {
        //check if coral is in hand
        //move elevator into the L4 position
        //move the wrist into the L4 position
        //expel the coral
    }

    public void expelCoral() {
        //expel coral
    }
    
    public void intakeAlgaeLow() {
        //check if coral or algae are in hand
        //move the elevator into algae low reef position
        //move the wrist into algae low reef position
        //intake the algae
    }

    public void intakeAlgaeHigh() {
        //check if coral or algae is in hand
        //move the elevator into algae high reef position
        //move the wrist into algae high reef position
        //intake the algae
    }
    
    public void scoreAlgaeNet() {
        //check if algae is in hand
        //move the elevator into net scoring position
        //move the wrist into the net scoring position
        //expel the algae at net scoring speed
    }

    public void scoreAlgaeProcessor() {
        //check if algae is in hand
        //move the elevator into processor scoring position
        //move the wrist into processor scoring position
        //expel the algae into the processor
    }

    public void expelAlgae() {
        //check if algae is in the hand
        //move hand to home (lowest possible) position
        //move wrist parallel to the ground
        //expel the algae at processor speed
    }

}
