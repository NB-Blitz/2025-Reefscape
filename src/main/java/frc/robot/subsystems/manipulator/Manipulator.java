package frc.robot.subsystems.manipulator;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
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
        //check if algae or coral is already in hand, run if not
        if(!robotHand.algaeInPosition() && !robotHand.coralInPosition()) {
            //put elevator into the coral station position
            //move the wrist into the coral station position 
            //intake coral using hand
            robotHand.intakeCoral();
        }
    }

    public void moveToL1() {
        //check if coral is in hand and algae not in hand
        if(robotHand.coralInPosition() && !robotHand.algaeInPosition()) {
            //move elevator into the L1 position
            //move the wrist into the L1 position
        }
    }

    public void moveToL2() {
        //check if coral is in hand
        if(robotHand.coralInPosition() && !robotHand.algaeInPosition()) {
            //move elevator into the L2/L3 position
            //move the wrist into the L2/L3 position
        }
    }

    public void moveToL3() {
        //check if coral is in hand
        if(robotHand.coralInPosition() && !robotHand.algaeInPosition()) {
            //move elevator into the L2/L3 position
            //move the wrist into the L2/L3 position
        }
    }

    public void moveToL4() {
        //check if coral is in hand
        if(robotHand.coralInPosition() && !robotHand.algaeInPosition()) {
            //move elevator into the L4 position
            //move the wrist into the L4 position
        }
    }

    public void expelCoral() {
        robotHand.expelCoral();
    }
    
    public void intakeAlgaeLow() {
        //check if coral or algae are not in hand to run
        if(!robotHand.algaeInPosition() && !robotHand.coralInPosition()) {
            //move the elevator into algae low reef position
            //move the wrist into algae low reef position
            //intake the algae
            robotHand.intakeAlgae();
        }
    }

    public void intakeAlgaeHigh() {
        ///check if coral or algae are not in hand to run
        if(!robotHand.algaeInPosition() && !robotHand.coralInPosition()) {
            //move the elevator into algae high reef position
            //move the wrist into algae high reef position
            //intake the algae
            robotHand.intakeAlgae();
        }
    }
    
    public void moveAlgaeNet() {
        //check if algae is in hand
        if(robotHand.algaeInPosition()) {
            //move the elevator into net scoring position
            //move the wrist into the net scoring position
        }
    }

    //expel the algae at net scoring speed
    public void expelAlgaeNet() {
        robotHand.expelAlgaeNet();
    }

    public void moveAlgaeProcessor() {
        //check if algae is in hand
        if(robotHand.algaeInPosition()) {

        }
        //move the elevator into processor scoring position
        //move the wrist into processor scoring position
    }

    //expel the algae into the processor
    public void expelAlgaeProcessor() {
        robotHand.expelAlgaeProcessor();
    }

    public void expelAlgae() {
        //check if algae is in the hand
        if(robotHand.algaeInPosition()) {
            //move hand to home (lowest possible) position
            //move wrist parallel to the ground
            //expel the algae at processor speed
            expelAlgaeProcessor();
        }
    }

}
