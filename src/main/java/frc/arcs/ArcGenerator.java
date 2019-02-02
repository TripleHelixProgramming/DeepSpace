package frc.arcs;

import static java.util.Arrays.asList;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.trajectory.WaypointSequence.Waypoint;
import com.team319.trajectory.AbstractBobPathCreator;
import com.team319.trajectory.BobPath;
import com.team319.trajectory.SrxTranslatorConfig;

public class ArcGenerator extends AbstractBobPathCreator {

    private static double robotWidthInFeet = 33.0 / 12.0;
	private static double robotLengthInFeet = 39.0 / 12.0;

	// This point and points like it can be used when there are common starting locatons for the robot
	// Remember that paths should be generated from the center point of the robot
	public static Waypoint startingPoint = new Waypoint(robotLengthInFeet / 2.0, 45.5 / 12.0, 0, 0, 0);
	
	SrxTranslatorConfig config = new SrxTranslatorConfig();
    
    public static void main(String[] args) {
        new ArcGenerator().generatePaths();
	}
	
	private ArcGenerator() {
		config.max_acc = 5.0; // Maximum acceleration in FPS
		config.max_vel = 7.0; // Maximum velocity in FPS
		config.wheel_dia_inches = 5.0;
		config.scale_factor = 1.48; // Used to adjust for a gear ratio and or distance tuning
		config.encoder_ticks_per_rev = 480; // Count of ticks on your encoder
		config.robotLength = 39; // Robot length in inches, used for drawing the robot
		config.robotWidth = 33; // Robot width in inches, used for drawing the robot
		config.highGear = true;
	}

	@Override
    protected List<BobPath> getArcs() {
		List<BobPath> paths = new ArrayList<>();
		paths.addAll(getConfigArcs());
		paths.addAll(generateTeamPaths());
        return paths;
	}

	/**
	 * Use this method to generate team paths. You can create more methods like this one to organize your path, 
	 * just make sure to add the method call to the returned list in getArcs()
	 * @return the list of team paths to generate
	 */
	private List<BobPath> generateTeamPaths() {
		 // Create a path with the name of "Example", this will generate a file named ExampleArc
		 BobPath exampleArc = new BobPath(config, "Example");
		 // Set the first point to the starating point, this be done with any of the addWaypoint methods
		 // positive X is forward, positive Y is left, units are in feet and degrees
		 exampleArc.addWaypoint(startingPoint);
		 // Add the next point that 3 ft forward, and doesn't turn, it also has a max speed of 5 FPS, 
		 // it will arrive at this location going 2 FPS
		 exampleArc.addWaypointRelative(3, 0, 0, 2, 5);
		 // Add the next point to be an additional 5 feet forward and 5 feet to the left with max speed of 2 FPS,
		 // it  will arrive at this locaton going 0 FPS 
		 exampleArc.addWaypointRelative(5, 5, 0, 0, 2);

		 BobPath straight = new BobPath(config, "Straight10Feet");
		 straight.addWaypoint(startingPoint);
		 straight.addWaypointRelative(10, 0, 0, 0, 5);

		 BobPath backwards = new BobPath(config, "StraightBack10Feet", true);
		 backwards.addWaypoint(startingPoint);
		 backwards.addWaypointRelative(10, 0, 0, 0, 5);

		 BobPath forwardLeft = new BobPath(config, "ForwardLeft");
		 forwardLeft.addWaypoint(startingPoint);
		 forwardLeft.addWaypoint(15, 5, 30, 5, 7);
		 forwardLeft.addWaypoint(20, 10, 0, 0, 5);

		 BobPath figure8 = new BobPath(config, "Figure8");
		 figure8.addWaypoint(startingPoint);
		 figure8.addWaypoint(10, 10, 45, 6, 6);
		 figure8.addWaypoint(15, 15, 0, 6, 6);
		 figure8.addWaypoint(20, 10, -89.99, 0, 6);
	
		 
		 return asList(exampleArc, straight, backwards, forwardLeft, figure8); // return asList(path1, path2, path3, ...);
	}
	
	
	/**
	 * Generate the configuration arcs, distance, turning, and speed
	 * DistanceScaling - This path will run 3 feet forward and stop. To tune this
	 * adjust the scaling factor until the robot stops at exactly 3 feet.
	 * TurnScaling - This path will run 3 feet forward and 3 feet to the left, this will 
	 * end at 90 degrees. This path can be used when tuning your heading loop for arc mode.
	 * SpeedTesting - This path will drive 3 feet forward and 3 feet to the left at 3 FPS,
	 * then drive another 3 feed forward and 3 feet to the left. This path will end with 
	 * the robot 6 feet to the left of it's starting position facing the oppostite direction.
	 */
	private List<BobPath> getConfigArcs() {
		BobPath distanceScaling = new BobPath(config, "DistanceScaling");
		distanceScaling.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		distanceScaling.addWaypointRelative(2, 0, 0, 0, 2);

		BobPath turnScaling = new BobPath(config, "TurnScaling");
		turnScaling.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		turnScaling.addWaypointRelative(3, 3, 89.99, 0, 2);

		BobPath speedTesting = new BobPath(config, "SpeedTesting");
		speedTesting.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		speedTesting.addWaypointRelative(3, 3, 89.99, 3, 5);
		speedTesting.addWaypointRelative(-3, 3, 89.99, 0, 3);

		return asList(distanceScaling, turnScaling, speedTesting);
	}
}