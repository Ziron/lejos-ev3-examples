
// package *add package name here*;

import lejos.hardware.Button;

public class DriveSquare extends EV3Skeleton {
    /**
     * Main function of program.
     */
    public static void main(String[] args) {
        initRobot();
        initPilot();

        System.out.println("Press any button to start!");
        Button.waitForAnyPress();
            
        // Add your code below here
        
        // Drive in a 0.5 meter square
        for (int i = 0; i < 4; i++) {
            pilot.travel(0.5);
            pilot.rotate(90);
        }
    }
}
