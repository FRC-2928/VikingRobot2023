package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Limelight utility is responsible for I/O with both Limelight 3
 * Feeds turret limelight to flywheel/hood/turret and operator shuffleboard
 * Feeds base limelight to intake vision tracking and driver shuffleboard
 */
public class Limelight {
	// Pulls values from network tables
	private NetworkTable m_limelightNI = NetworkTableInstance.getDefault().getTable("limelight");

	// Creates variables to assign
	private double horizontalOffset;
	private double verticalOffset;
	private double area;
	private double targetDistance;
	private double skew;


	private boolean m_targetFound;


	// -----------------------------------------------------------
	// Initialization
	// -----------------------------------------------------------
	public Limelight() {
		this.setStream(0);
	}

	// -----------------------------------------------------------
	// Control Input
	// -----------------------------------------------------------
	public void updateReadings() {
		this.horizontalOffset = this.getHorizontalOffset();
		this.verticalOffset = this.getVerticalOffset();
		//this.m_targetDistance = this.getTargetDistance();
		this.m_targetFound = this.isTargetFound();
		this.skew = this.getSkew();
	}

	public void setStream(int stream) {
		this.m_limelightNI.getEntry("stream").setNumber(stream);
	}
	
	// -----------------------------------------------------------
	// System State
	// -----------------------------------------------------------
	public double getSkew() {
		return this.m_limelightNI.getEntry("ts").getDouble(0);
	}

	public double getHasValidTargets(){
		return this.m_limelightNI.getEntry("tv").getDouble(0);
	}

	public double[] getPose(){
		double[] pose = this.m_limelightNI.getEntry("botpose").getDoubleArray(new double[6]);
		if(pose.length == 0) {
			return new double[6]; 
		}
		else {
			return pose;
		}
	}

	public double getPoseX(){
		double pose = this.m_limelightNI.getEntry("botpose").getDouble(0);
		return pose;
	}

	// public double getTargetDistance(){
	//   double h = (LimelightConstants.kHighGoalHeight - LimelightConstants.kHighLimelightHeight) / 12;
	//   return h/Math.tan(Math.toRadians(getVerticalOffset() + LimelightConstants.kHighLimelightMountAngle));
	// }

	public double getHorizontalOffset() {
		NetworkTableEntry tx = this.m_limelightNI.getEntry("tx");
		this.horizontalOffset = tx.getDouble(0.0);
		return this.horizontalOffset;
	}

	public double getVerticalOffset() {
		NetworkTableEntry ty = this.m_limelightNI.getEntry("ty");
		this.verticalOffset = ty.getDouble(0.0);
		return this.verticalOffset;
	}

	public double getArea() {
		return this.area;
	}

	public boolean isTargetFound() {
		if(this.m_limelightNI.getEntry("tv").getDouble(0.0) == 0.0) {
			this.m_targetFound = false;
		}
		else{
			this.m_targetFound = true;
		}
		return this.m_targetFound;
	}
}

