package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Limelight utility is responsible for I/O with both Limelight 3
 * Feeds turret limelight to flywheel/hood/turret and operator shuffleboard
 * Feeds base limelight to intake vision tracking and driver shuffleboard
 */
public class Limelight {
	// Pulls values from network tables
	private NetworkTable m_limelightNI = NetworkTableInstance.getDefault().getTable("limelight");

	
	// -----------------------------------------------------------
	// Initialization
	// -----------------------------------------------------------
	public Limelight() {
		this.setStream(0);
	}

	// -----------------------------------------------------------
	// Control Input
	// -----------------------------------------------------------
	
	public void setStream(int stream) {
		this.m_limelightNI.getEntry("stream").setNumber(stream);
	}
	
	// -----------------------------------------------------------
	// System State
	// -----------------------------------------------------------
	
	// Whether the limelight has any valid targets (0 or 1)
	public boolean getHasValidTargets(){
		return this.m_limelightNI.getEntry("tv").getDouble(0) == 1;
	}

	// ------------------------------------------------------------------------
	// Poses using reflective tape
	// ------------------------------------------------------------------------

	// Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
	public double getHorizontalOffset() {
		return this.m_limelightNI.getEntry("tx").getDouble(0.0);
	}

	// Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
	public double getVerticalOffset() {
		return this.m_limelightNI.getEntry("ty").getDouble(0.0);
	}

	// Target Area (0% of image to 100% of image)
	public double getArea() {
		return this.m_limelightNI.getEntry("ta").getDouble(0.0);
	}

	public double getSkew() {
		return this.m_limelightNI.getEntry("ts").getDouble(0);
	}

	// ------------------------------------------------------------------------
	// Localization Poses using AprilTags
	// ------------------------------------------------------------------------

	// Robot transform in 3D field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
	public Pose3d getPose3d(){
		double[] pose = this.m_limelightNI.getEntry("botpose").getDoubleArray(new double[6]);
		if(pose.length == 0) {
			return new Pose3d(); 
		}
		else {
			return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), 
							  new Rotation3d(pose[3] * Math.PI / 180, 
							  				 pose[4] * Math.PI / 180, 
											 pose[5] * Math.PI / 180));
		}
	}

	// Robot transform in 2D field-space. Translation (X,Y) Rotation(Z)
	public Pose2d getPose2d(){
		return getPose3d().toPose2d();
	}

	// ---------------------------------------------------------------------
	// Blue Pose
	// ---------------------------------------------------------------------
	// Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(X,Y,Z)
	public Pose3d getBluePose3d(){
		double[] pose = this.m_limelightNI.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
		if(pose.length == 0) {
			return new Pose3d(); 
		}
		else {
			return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), 
							  new Rotation3d(pose[3] * Math.PI / 180, 
							  				 pose[4] * Math.PI / 180, 
											 pose[5] * Math.PI / 180));
		}
	}

	public Pose2d getBluePose2d(){
		return getBluePose3d().toPose2d();
	}

	// public Pose2d getBluePose2d(){
	// 	double[] pose = this.m_limelightNI.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
	// 	SmartDashboard.putNumber("pose 0", pose[0]);
	// 	SmartDashboard.putNumber("pose 1", pose[1]);
	// 	SmartDashboard.putNumber("pose 5", pose[5]);
	// 	if(pose.length == 0) {
	// 		return new Pose2d(); 
	// 	}
	// 	else {
	// 		return new Pose2d(pose[0], pose[1], 
	// 						  new Rotation2d(pose[5] * Math.PI / 180));
	// 	}
	// }

	// Robot transform in field-space. Translation (X)
	public double getPoseX(){
		return getPose2d().getX();
	}

	// Robot transform in field-space. Translation (Y)
	public double getPoseY(){
		return getPose2d().getY();
	}

	// ---------------------------------------------------------------------
	// Red Pose
	// ---------------------------------------------------------------------
	public Pose2d getRedPose2d(){
		return getRedPose3d().toPose2d();
	}

	// Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(X,Y,Z)
	public Pose3d getRedPose3d(){
		double[] pose = this.m_limelightNI.getEntry("botpose_wpired").getDoubleArray(new double[6]);
		if(pose.length == 0) {
			return new Pose3d(); 
		}
		else {
			return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), 
							  new Rotation3d(pose[3]  * Math.PI / 180, 
							  				 pose[4] * Math.PI / 180, 
											 pose[5] * Math.PI / 180));
		}
	}

	// Robot transform in field-space. Translation (X)
	public double getPoseXRed(){
		return getRedPose3d().getX();
	}

	// Robot transform in field-space. Translation (X)
	public double getPoseXBlue(){
		return getBluePose3d().getX();
	}

	// ------------------------------------------------------------------------
	// Pose relative to the primary in-view AprilTag
	// ------------------------------------------------------------------------

	// 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
	public Pose3d getRobotTagPose3d() {
		double[] pose = this.m_limelightNI.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
		if(pose.length == 0) {
			return new Pose3d(); 
		}
		else {
			return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), 
							  new Rotation3d(pose[3] * Math.PI / 180, 
							  				 pose[4] * Math.PI / 180, 
											 pose[5] * Math.PI / 180));
		}
	}

	// 3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
	public Pose3d getCameraTagPose3d() {
		double[] pose = this.m_limelightNI.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
		if(pose.length == 0) {
			return new Pose3d(); 
		}
		else {
			return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), 
							  new Rotation3d(pose[3] * Math.PI / 180, 
							  				 pose[4] * Math.PI / 180, 
											 pose[5] * Math.PI / 180));
		}
	}
	
}

