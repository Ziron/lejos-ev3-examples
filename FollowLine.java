
// package *add package name here*;

import lejos.hardware.Button;
import lejos.utility.Delay;

public class FollowLine extends EV3Skeleton {
    /**
     * Main function of program.
     */
    public static void main(String[] args) {
        initRobot();
        initPilot();

        System.out.println("Press any button to start!");
        Button.waitForAnyPress();
            
        // Add your code below here
        
        double lineThreshold = 0.35;
        
        while(true) {
            if (colorSensor.getReflectedRed() < lineThreshold) {
                // On line, turn sharply to the left
                pilot.arcForward(0.05);
            } else {
                // Off line, turn slightly to the right
                pilot.arcForward(-0.2);
            }
            
            // A bit of delay to allow the robot time to move
            Delay.msDelay(100);
        }
    }
}
