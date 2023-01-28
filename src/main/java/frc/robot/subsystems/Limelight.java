// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import frc.robot.Constants.LimelightConstants;

/**
 * Limelight utility is responsible for I/O with both Limelight 2+
 * Feeds turret limelight to flywheel/hood/turret and operator shuffleboard
 * Feeds base limelight to intake vision tracking and driver shuffleboard
 */
public class Limelight{
  //Pulls values from network tables
  private NetworkTable m_limelightNI = NetworkTableInstance.getDefault().getTable("limelight");

  //Creates variables to assign
  private double m_horizontalOffset;
  private double m_verticalOffset;
  private double m_area;
  private double m_targetDistance;
  private double m_skew;


  private boolean m_targetFound;


  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public Limelight() {
    setStream(0);
  }

  public LimelightData getLimelightData() {
    updateReadings();
    return new LimelightData(m_horizontalOffset, m_verticalOffset, m_targetDistance, m_targetFound, m_skew);
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void updateReadings() {
    m_horizontalOffset = getHorizontalOffset();
    m_verticalOffset = getVerticalOffset();
    //m_targetDistance = getTargetDistance();
    m_targetFound = isTargetFound();
    m_skew = getSkew();
  }

  public void setStream(int stream) {
    m_limelightNI.getEntry("stream").setNumber(stream);
  }
  
  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  public double getSkew() {
    return m_limelightNI.getEntry("ts").getDouble(0);
  }

  public double getHasValidTargets(){
    return m_limelightNI.getEntry("tv").getDouble(0);
  }

  public double[] getPose(){
    double[] pose = m_limelightNI.getEntry("botpose").getDoubleArray(new double[6]);
    if(pose.length == 0) {
      return new double[6]; 
    }
    else {
      return pose;
    }
  }

  public double getPoseX(){
    double pose = m_limelightNI.getEntry("botpose").getDouble(0);
    return pose;
  }

  // public double getTargetDistance(){
  //   double h = (LimelightConstants.kHighGoalHeight - LimelightConstants.kHighLimelightHeight) / 12;
  //   return h/Math.tan(Math.toRadians(getVerticalOffset() + LimelightConstants.kHighLimelightMountAngle));
  // }

  public double getHorizontalOffset() {
    NetworkTableEntry tx = m_limelightNI.getEntry("tx");
    m_horizontalOffset = tx.getDouble(0.0);
    return m_horizontalOffset;
  }

  public double getVerticalOffset() {
    NetworkTableEntry ty = m_limelightNI.getEntry("ty");
    m_verticalOffset = ty.getDouble(0.0);
    return m_verticalOffset;
  }

  public double getArea() {
    return m_area;
  }

  public boolean isTargetFound() {
    if (m_limelightNI.getEntry("tv").getDouble(0.0) == 0.0) {
      m_targetFound = false;
    }
    else{
      m_targetFound = true;
    }
    return m_targetFound;
  }
}

