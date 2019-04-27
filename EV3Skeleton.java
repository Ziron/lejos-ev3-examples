
// package *add package name here*;


import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class EV3Skeleton {

    /**
     * Diameter of a standard EV3 wheel in meters.
     */
    static final double WHEEL_DIAMETER = 0.056;
    /**
     * Distance between center of wheels in meters.
     */
    static final double WHEEL_SPACING = 0.12;

    // These variables are initialized by initRobot()
    static Brick brick;
    static EV3MediumRegulatedMotor clawMotor;
    static EV3LargeRegulatedMotor leftMotor;
    static EV3LargeRegulatedMotor rightMotor;
    static EV3TouchSensorWrapper touchSensor;
    static EV3GyroSensorWrapper gyroSensor;
    static EV3ColorSensorWrapper colorSensor;
    static EV3UltrasonicSensorWrapper distanceSensor;

    // These variables are initalized by initPilot()
    static MovePilot pilot;
    static PoseProvider poseProvider;

    /**
     * Main function of program.
     */
    public static void main(String[] args) {
        initRobot();
        initPilot();

        System.out.println("Press any button to start!");
        Button.waitForAnyPress();

        // Add your code below here
        
        

    }

    /**
     * Instantiates a brick and all the motors and sensors of the robot.
     */
    public static void initRobot() {
        if (brick != null) {
            // Already initialized
            return;
        }
        Button.LEDPattern(5); // Flashing red
        System.out.println("Initializing...");

        brick = BrickFinder.getDefault();

        clawMotor = new EV3MediumRegulatedMotor(brick.getPort("A"));
        leftMotor = new EV3LargeRegulatedMotor(brick.getPort("B"));
        rightMotor = new EV3LargeRegulatedMotor(brick.getPort("C"));

        // Initialize sensors in separate threads because each take a lot of time.
        Thread touchThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (touchSensor == null) {
                    try {
                        touchSensor = new EV3TouchSensorWrapper(brick.getPort("S1"));
                    } catch (IllegalArgumentException e) {
                        System.err.println("Touch sensor: " + e.getMessage());
                    }
                }
            }
        });
        Thread gyroThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (gyroSensor == null) {
                    try {
                        gyroSensor = new EV3GyroSensorWrapper(brick.getPort("S2"));
                    } catch (IllegalArgumentException e) {
                        System.err.println("Gyro sensor: " + e.getMessage());
                    }
                }
            }
        });
        Thread colorThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (colorSensor == null) {
                    try {
                        colorSensor = new EV3ColorSensorWrapper(brick.getPort("S3"));
                    } catch (IllegalArgumentException e) {
                        System.err.println("Color sensor: " + e.getMessage());
                    }
                }
            }
        });
        Thread distanceThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (distanceSensor == null) {
                    try {
                        distanceSensor = new EV3UltrasonicSensorWrapper(brick.getPort("S4"));
                    } catch (IllegalArgumentException e) {
                        System.err.println("Ultrasonic sensor: " + e.getMessage());
                    }
                }
            }
        });

        touchThread.start();
        gyroThread.start();
        colorThread.start();
        distanceThread.start();

        // Wait for sensors to be initialized.
        try {
            touchThread.join();
            gyroThread.join();
            colorThread.join();
            distanceThread.join();
        } catch (InterruptedException e) {
            System.err.println(e.getMessage());
        }

        Button.LEDPattern(1); // Steady green
        Sound.beepSequenceUp();
        System.out.println("Ready!");
    }

    /**
     * Instantiates a MovePilot and PoseProvider.
     * <br>
     * Don't call this function if you plan to use leftMotor and rightMotor
     * directly to control the robot.
     */
    public static void initPilot() {
        if (pilot != null) {
            // Already initialized
            return;
        }
        Wheel leftWheel = WheeledChassis.modelWheel(leftMotor, WHEEL_DIAMETER).offset(WHEEL_SPACING / 2);
        Wheel rightWheel = WheeledChassis.modelWheel(rightMotor, WHEEL_DIAMETER).offset(-WHEEL_SPACING / 2);

        Chassis chassis = new WheeledChassis(new Wheel[]{leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);

        pilot = new MovePilot(chassis);
        poseProvider = new OdometryPoseProvider(pilot);

        // Set default speed to 0.03 m/s and acceleration to 0.5 m/s^2
        pilot.setLinearSpeed(0.03);
        pilot.setLinearAcceleration(0.5);

        // Set default turn speed to 180 deg/s and acceleration to 1800 deg/s^2
        pilot.setAngularSpeed(180);
        pilot.setAngularAcceleration(1800);
    }

    /**
     * Wrapper class to allow easier use of EV3TouchSensor.
     */
    public static class EV3TouchSensorWrapper extends EV3TouchSensor {

        private final SampleProvider sampleProvider;
        private final float[] samples;

        public EV3TouchSensorWrapper(Port port) {
            super(port);

            try {
                sampleProvider = super.getTouchMode();
            } catch (IllegalArgumentException e) {
                super.close();
                throw e;
            }
            samples = new float[1];
        }

        /**
         * Detects if the sensor button is pressed.
         *
         * @return Whether the sensor button is pressed.
         */
        public boolean isPressed() {
            sampleProvider.fetchSample(samples, 0);

            return samples[0] != 0;
        }
    }

    /**
     * Wrapper class to allow easier use of EV3GyroSensor.
     */
    public static class EV3GyroSensorWrapper extends EV3GyroSensor {

        private final SampleProvider sampleProvider;
        private final float[] samples;

        public EV3GyroSensorWrapper(Port port) {
            super(port);

            try {
                sampleProvider = super.getAngleAndRateMode();
            } catch (IllegalArgumentException e) {
                super.close();
                throw e;
            }

            samples = new float[2];
        }

        /**
         * Measures the orientation of the sensor in respect to its start
         * orientation.
         * <br>
         * The start position can be set to the current position using the
         * reset() method of the sensor.
         *
         * @return The orientation (in Degrees) of the sensor in respect to its
         * start position.
         */
        public double getAngle() {
            sampleProvider.fetchSample(samples, 0);

            return samples[1];
        }

        /**
         * Measures angular velocity of the sensor.
         *
         * @return The angular velocity (in Degrees / second) of the sensor.
         */
        public double getAngularVelocity() {
            sampleProvider.fetchSample(samples, 0);

            return samples[0];
        }
    }

    /**
     * Wrapper class to allow easier use of EV3ColorSensor.
     */
    public static class EV3ColorSensorWrapper extends EV3ColorSensor {

        private final SampleProvider sampleProviderRed;
        private final SampleProvider sampleProviderRGB;
        private final SampleProvider sampleProviderColor;
        private final float[] samples;

        public EV3ColorSensorWrapper(Port port) {
            super(port);

            try {
                sampleProviderRed = super.getRedMode();
                sampleProviderRGB = super.getRGBMode();
                sampleProviderColor = super.getColorIDMode();
            } catch (IllegalArgumentException e) {
                super.close();
                throw e;
            }

            samples = new float[3];
        }

        /**
         * Measures the level of reflected light from the sensors RED LED.
         *
         * @return A value containing the intensity level (Normalized between 0
         * and 1) of reflected light.
         *
         */
        public double getReflectedRed() {
            sampleProviderRed.fetchSample(samples, 0);

            return samples[0];
        }

        /**
         * Measures the level of reflected red, green and blue light when
         * illuminated by a white light source.
         *
         * @return The sample contains 3 elements containing the intensity level
         * (Normalized between 0 and 1) of red, green and blue light
         * respectivily.
         */
        public double[] getReflectedRGB() {
            sampleProviderRGB.fetchSample(samples, 0);

            return new double[]{samples[0], samples[1], samples[2]};
        }

        /**
         * Measures the color ID of a surface. The sensor can identify 8 unique
         * colors (NONE, BLACK, BLUE, GREEN, YELLOW, RED, WHITE, BROWN).
         *
         * @return A value corresponding to identified color. See
         * {@link lejos.robotics.Color}
         */
        public int getColor() {
            sampleProviderColor.fetchSample(samples, 0);

            return (int) samples[0];
        }
    }

    /**
     * Wrapper class to allow easier use of EV3UltrasonicSensor.
     */
    public static class EV3UltrasonicSensorWrapper extends EV3UltrasonicSensor {

        private final SampleProvider sampleProvider;
        private final float[] samples;

        public EV3UltrasonicSensorWrapper(Port port) {
            super(port);

            try {
                sampleProvider = super.getDistanceMode();
            } catch (IllegalArgumentException e) {
                super.close();
                throw e;
            }
            samples = new float[1];
        }

        /**
         * Measures distance to an object in front of the sensor
         *
         * @return The distance (in meters) to an object in front of the sensor.
         */
        public double getDistance() {
            sampleProvider.fetchSample(samples, 0);

            return samples[0];
        }
    }
}
