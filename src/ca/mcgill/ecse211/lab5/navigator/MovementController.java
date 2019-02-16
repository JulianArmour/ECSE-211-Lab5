package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Provides methods for controlling the robot's motions and navigation instructions.
 * 
 * @author Julian Armour, Alice Kazarine
 * @since 06-02-2018
 * @version 1.2
 */
public class MovementController {
    private static final int ROTATE_SPEED = 45;
    private static final int FORWARD_SPEED = 150;
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private Odometer odometer;
    private double wheelRadius;
    private double track;

    /**
     * Creates a MovementController object.
     * 
     * @param leftMotor     The motor for the left wheel
     * @param rightMotor    The motor for the right wheel
     * @param wheelRadius   The radius of the wheels
     * @param track         The track length for the wheels
     * @param odometer      The odometer used
     */
    public MovementController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double wheelRadius,
            double track, Odometer odometer) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.odometer = odometer;
        this.wheelRadius = wheelRadius;
        this.track = track;
    }

    /**
     * Turns the robot towards an absolute (i.e. not relative) angle on the platform
     * 
     * @param theta The angle in degrees the robot will rotate to.
     */
    public void turnTo(double theta) {
        // angle component of the odometer
        double heading = odometer.getXYT()[2];
        // cyclic angle distance
        double dT = ((theta - heading + 360) % 360);

        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);

        // Turn the smallest angle
        if (dT < 180) {
            rotateAngle(dT, true);
        } else {
            rotateAngle(360 - dT, false);
        }
    }

    /**
     * Rotates the robot by the specified angle. The thread waits until the rotation
     * is complete.
     * 
     * @param theta         The number of degrees to turn
     * @param turnClockwise Specifies if the robot should turn clockwise. If false, then the
     *                      robot will turn counter-clockwise.
     */
    public void rotateAngle(double theta, boolean turnClockwise) {
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        if (turnClockwise) {
            leftMotor.rotate(convertAngle(wheelRadius, track, theta), true);
            rightMotor.rotate(-convertAngle(wheelRadius, track, theta), false);
        } else {
            leftMotor.rotate(-convertAngle(wheelRadius, track, theta), true);
            rightMotor.rotate(convertAngle(wheelRadius, track, theta), false);
        }
    }
    
    /**
     * Rotates the robot by the specified angle. The thread may or may not wait until the
     * rotation is complete depending on immediateReturn.
     * 
     * @param theta             The number of degrees to turn
     * @param turnClockwise     Specifies if the robot should turn clockwise. If false, then the
     *                          robot will turn counter-clockwise.
     * @param immediateReturn   If true, do not wait for the move to complete
     */
    public void rotateAngle(double theta, boolean turnClockwise, boolean immediateReturn) {
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        if (turnClockwise) {
            leftMotor.rotate(convertAngle(wheelRadius, track, theta), true);
            rightMotor.rotate(-convertAngle(wheelRadius, track, theta), immediateReturn);
        } else {
            leftMotor.rotate(-convertAngle(wheelRadius, track, theta), true);
            rightMotor.rotate(convertAngle(wheelRadius, track, theta), immediateReturn);
        }
    }
    

    /**
     * Causes the robot to move forward the specified distance
     * 
     * @param distance The distance in centimeters the robot should move
     */
    public void driveDistance(double distance) {
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);

        leftMotor.rotate(convertDistance(wheelRadius, distance), true);
        rightMotor.rotate(convertDistance(wheelRadius, distance), false);
    }
    
    /**
     * Causes the robot to move forward the specified distance. Can cause the thread
     * not not wait for the move to complete if immediateReturn is true.
     * 
     * @param distance          The distance in centimeters the robot should move
     * @param immediateReturn   If true, do not wait for the move to complete
     */
    public void driveDistance(double distance, boolean immediateReturn) {
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);

        leftMotor.rotate(convertDistance(wheelRadius, distance), true);
        rightMotor.rotate(convertDistance(wheelRadius, distance), true);
    }
    
    /**
     * Causes the robot to drive forward until {@link #stopMotors()} is called.
     */
    public void driveForward() {
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);
        
        leftMotor.forward();
        rightMotor.forward();
    }
    
    /**
     * Causes the robot to stop.
     */
    public void stopMotors() {
        leftMotor.stop(true);
        rightMotor.stop(false);
    }

    /**
     * This method allows the conversion of a distance to the total rotation of each
     * wheel needed to cover that distance.
     * 
     * @param radius
     *            The radius of the robot's wheels
     * @param distance
     *            The distance to be converted to degrees
     * @return The angle the wheels should turn to cover the distance
     */
    private static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }
    
    /**
     * @param radius
     *            The radius of the wheels
     * @param width
     *            The distance between the wheels
     * @param angle
     *            The angle the robot should turn
     * @return The angle in degrees the wheels must rotate to turn the robot a
     *         certain angle
     */
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
    
    
    /**
     * Calculates the distance between position and destination.
     * 
     * @param Xi x-component of position i
     * @param Yi y-component of position i
     * @param Xf x-component of position f
     * @param Yf y-component of position f
     * @return the distance between the two vectors
     */
    public double calculateDistance(double Xi, double Yi, double Xf, double Yf) {
    	 double dx = Xf - Xi;
         double dy = Yf - Yf;
  
    	 double distanceToWaypoint = Math.sqrt(dx * dx + dy * dy);
    	 return distanceToWaypoint;
    	 //
    }
    
    /**
     * calculates the smallest angle between position to destination
     * 
     * @param Xi x-component of position i
     * @param Yi y-component of position i
     * @param Xf x-component of position f
     * @param Yf y-component of position f
     * @return The angle the robot should face to reach position f from position i
     */
    public double calculateAngle(double Xi,double Yi,double Xf,double Yf) {
    	 double dx = Xf - Xi;
         double dy = Yf - Yi;

         double angleToHead;

         if (dy == 0.0) {
             if (dx >= 0) {
                 angleToHead = 90.0;
             } else {
                 angleToHead = 270.0;
             }
         }
         // first quadrant (0-90)
         else if (dx >= 0.0 && dy > 0.0) {
             angleToHead = Math.toDegrees(Math.atan(dx / dy));
         }
         // second quadrant (270-360)
         else if (dx < 0.0 && dy > 0.0) {
             angleToHead = 360.0 + Math.toDegrees(Math.atan(dx / dy));
         }
         // third and fourth quadrant (90-270)
         else {
             angleToHead = 180.0 + Math.toDegrees(Math.atan(dx / dy));
         }
    	return angleToHead;
    }
    
    public void turnLeft(int MOTOR_SPEED, int delta) {
		leftMotor.setSpeed(MOTOR_SPEED - delta);
		rightMotor.setSpeed(MOTOR_SPEED + delta);
		leftMotor.forward();
		rightMotor.forward();
	}

	
	public void turnRight(int MOTOR_SPEED, int delta) {
		leftMotor.setSpeed(MOTOR_SPEED + delta);
		rightMotor.setSpeed(MOTOR_SPEED - delta);
		leftMotor.forward();
		rightMotor.forward();
	}
	
	public void travelCloseToOrigin(Odometer odo) {

        //double[] odoData = odo.getXYT();
        double angleToTurn = calculateAngle(odo.getXYT()[0], odo.getXYT()[1], -5.0, -5.0);
        System.out.println("ANGLE TO TURN: "+angleToTurn);
        turnTo(angleToTurn);

        // give the robot some time to stop
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        driveDistance(calculateDistance(odo.getXYT()[0], odo.getXYT()[1], -5.0, -5.0));
       
    }
	public void travelTo(Odometer odo,double Xf, double Yf) {
	
        double angleToTurn = calculateAngle(odo.getXYT()[0], odo.getXYT()[1], Xf, Yf);
       // System.out.println("ANGLE TO TURN: "+angleToTurn);
        turnTo(angleToTurn);

        // give the robot some time to stop
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        driveDistance(calculateDistance(odo.getXYT()[0], odo.getXYT()[1], Xf, Yf));
		
	}
}
