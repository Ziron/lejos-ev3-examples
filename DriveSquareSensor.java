
// package *add package name here*;

import lejos.hardware.Button;

public class DriveSquareSensor extends EV3Skeleton {
    /**
     * Main function of program.
     */
    public static void main(String[] args) {
        initRobot();
        initPilot();

        System.out.println("Press any button to start!");
        Button.waitForAnyPress();
            
        // Add your code below here
        
        // Drive until reaching an obstacle 0.2 meters in front and turn.
        for (int i = 0; i < 4; i++) {
            pilot.forward();
            while (distanceSensor.getDistance() > 0.2) {
                // Do nothing
            }
            pilot.rotate(90);
        }
    }
}
