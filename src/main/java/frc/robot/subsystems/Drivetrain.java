package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.sim.DrivebaseSimFX;
import frc.robot.subsystems.Transmission.GearState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class Drivetrain extends SubsystemBase {
	public final WPI_TalonFX leftLeader = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainLeftBackTalonFX);
	public final WPI_TalonFX rightLeader = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainRightBackTalonFX);
	public final WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainLeftFrontTalonFX);
	public final WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.CANBusIDs.DrivetrainRightFrontTalonFX);

  	private final Limelight limelight = new Limelight();

	private Supplier<Transmission.GearState> gearStateSupplier;

	public DifferentialDrive diffDrive;

	// Set up the BuiltInAccelerometer
	public WPI_Pigeon2 pigeon = new WPI_Pigeon2(Constants.CANBusIDs.kPigeonIMU);

	// Drivetrain kinematics, feed it width between wheels
	private SimpleMotorFeedforward feedForward;

	// Drivetrain odometry to keep track of our position on the field
	private DifferentialDriveOdometry odometry;

	private DifferentialDrivePoseEstimator poseEstimator;

	private final Field2d field2d = new Field2d();
	private final Field2d fieldEstimated = new Field2d();
	private final Field2d fieldLimelight = new Field2d();

	/* Object for simulated drivetrain. */	
	private DrivebaseSimFX driveSim = new DrivebaseSimFX(leftLeader, rightLeader, pigeon);

	// -----------------------------------------------------------
	// Initialization
	// -----------------------------------------------------------
	public Drivetrain(Supplier<Transmission.GearState> gearStateSupplier) {
		this.gearStateSupplier = gearStateSupplier;

		// Configure Talon motors
		this.configureMotors();

		this.diffDrive = new DifferentialDrive(leftLeader, rightLeader);

		this.feedForward = DrivetrainConstants.kFeedForward;

		this.resetEncoders();
		this.zeroGyro();

		// Start with default Pose2d(0, 0, 0)
		this.odometry = new DifferentialDriveOdometry(new Rotation2d(this.readYaw()), 0, 0);
		this.poseEstimator = new DifferentialDrivePoseEstimator(DrivetrainConstants.kDriveKinematics, new Rotation2d(this.readYaw()), 0, 0, this.getLimelightPose());

		this.field2d.setRobotPose(this.getEncoderPose());
		SmartDashboard.putData("Field", this.field2d);

		this.fieldEstimated.setRobotPose(getEstimatedPose());
		SmartDashboard.putData("Estimated Pose", this.fieldEstimated);

		this.fieldLimelight.setRobotPose(getLimelightPose());
		SmartDashboard.putData("Limelight Pose", this.fieldLimelight);
	}

	public void configureMotors() {
		// Configure the motors
		for(TalonFX fx : new TalonFX[] { this.leftLeader, this.leftFollower, this.rightLeader, this.rightFollower }) {
			// Reset settings for safety
			fx.configFactoryDefault();

			// Sets voltage compensation to 10, used for percent output
			fx.configVoltageCompSaturation(10);
			fx.enableVoltageCompensation(true);

			// Setting just in case
			fx.configNominalOutputForward(0);
			fx.configNominalOutputReverse(0);
			fx.configPeakOutputForward(1);
			fx.configPeakOutputReverse(-1);

			fx.configOpenloopRamp(0.1);

			// Setting deadband(area required to start moving the motor) to 1%
			fx.configNeutralDeadband(0.01);

			// Set to brake mode, will brake the motor when no power is sent
			fx.setNeutralMode(NeutralMode.Brake);

			/**
			 * Setting input side current limit (amps)
			 * 45 continious, 80 peak, 30 millieseconds allowed at peak
			 * 40 amp breaker can support above 40 amps for a little bit
			 * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds
			 * should be fine
			 */
			fx.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

			// Either using the integrated Falcon sensor or an external one, will change if
			// needed
			fx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		}

		// New Talon FX inverts. Would replace InvertType.InvertMotorOutput
		// this.leftLeader.setInverted(TalonFXInvertType.CounterClockwise);
		// this.rightLeader.setInverted(TalonFXInvertType.Clockwise);

		// Setting followers, followers don't automatically follow the Leader's inverts
		// so you must set the invert type to Follow the Leader
		this.leftFollower.setInverted(InvertType.FollowMaster);
		this.rightFollower.setInverted(InvertType.FollowMaster);

		this.leftFollower.follow(this.leftLeader, FollowerType.PercentOutput);
		this.rightFollower.follow(this.rightLeader, FollowerType.PercentOutput);

		this.rightLeader.setInverted(InvertType.InvertMotorOutput);
	}

	// -----------------------------------------------------------
	// Control Input
	// -----------------------------------------------------------
	public void halt() {
		this.tankDriveVolts(0, 0);
	}

	public void enableMotorSafety() {
		this.diffDrive.setSafetyEnabled(true);
	}

	public void disableMotorSafety() {
		this.diffDrive.setSafetyEnabled(false);
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		this.leftLeader.set(ControlMode.PercentOutput, leftVolts / 12);
		this.rightLeader.set(ControlMode.PercentOutput, rightVolts / 12);

		// Feed motor safety to assert that we're in control
		this.diffDrive.feed();
	}
	public Pose3d readTarget(int number){
		if(number<10){
				return FieldConstants.targets.get(number);
		}
		if(number<9 & number<19){
				return FieldConstants.targets2.get(number);
		}
		if(number>18 & number < 28){
				return FieldConstants.targets3.get(number);
		}
		else{
			return null;
		}

}
	public void zeroGyro() {
		this.pigeon.setYaw(0);
		this.pigeon.reset();
	}

	public void resetEncoders() {
		this.leftLeader.setSelectedSensorPosition(0);
		this.rightLeader.setSelectedSensorPosition(0);
	}

	public void resetOdometry(Pose2d pose) {
		this.resetEncoders();
		this.odometry.resetPosition(this.read2dRotation(), 0, 0, pose);
	}

//   
	public void setOutputMetersPerSecond(double rightMetersPerSecond, double leftMetersPerSecond) {
		// Log.writeln("right m/s" + rightMetersPerSecond);
		// Calculate feedforward for the left and right wheels.
		double leftFeedForward = this.feedForward.calculate(leftMetersPerSecond);
		double rightFeedForward = this.feedForward.calculate(rightMetersPerSecond);

		SmartDashboard.putNumber("left meters per sec", leftMetersPerSecond);
		SmartDashboard.putNumber("right meters per sec", rightMetersPerSecond);

		// Convert meters per second to encoder ticks per second
		GearState gearState = this.gearStateSupplier.get();
		double leftVelocityTicksPerSec = this.wheelRotationsToEncoderTicks(
			this.metersToWheelRotations(leftMetersPerSecond),
			gearState
		);
		double rightVelocityTicksPerSec = this.wheelRotationsToEncoderTicks(
			this.metersToWheelRotations(rightMetersPerSecond),
			gearState
		);

		SmartDashboard.putNumber("left velocity ticks per second", leftVelocityTicksPerSec);
		SmartDashboard.putNumber("right velocity ticks per second", rightVelocityTicksPerSec);

		this.leftLeader.set(ControlMode.Velocity,
				leftVelocityTicksPerSec / 10.0,
				DemandType.ArbitraryFeedForward,
				leftFeedForward / DrivetrainConstants.k_MaxVolts);
		this.rightLeader.set(ControlMode.Velocity,
				rightVelocityTicksPerSec / 10.0,
				DemandType.ArbitraryFeedForward,
				rightFeedForward / DrivetrainConstants.k_MaxVolts);

		this.diffDrive.feed();
	}

	// -----------------------------------------------------------
	// System State
	// -----------------------------------------------------------
	public double motorRotationsToWheelRotations(double motorRotations, Transmission.GearState gearState) {
		if(gearState == Transmission.GearState.HIGH) {
			return motorRotations / (DrivetrainConstants.encoderCPR * DrivetrainConstants.highGearRatio);
		} else {
			return motorRotations / (DrivetrainConstants.encoderCPR * DrivetrainConstants.lowGearRatio);
		}
	}

	public double wheelRotationsToMeters(double wheelRotations) {
		return DrivetrainConstants.kWheelDiameterMeters * Math.PI * wheelRotations;
	}

	// Encoder ticks to meters
	public double encoderTicksToMeters(double encoderTicks) {
		GearState gearState = this.gearStateSupplier.get();
		return this.wheelRotationsToMeters(this.motorRotationsToWheelRotations(encoderTicks, gearState));
	}

	public double getLeftDistanceMeters() {
		return this.encoderTicksToMeters(this.leftLeader.getSelectedSensorPosition());
	}

	public double getRightDistanceMeters() {
		return this.encoderTicksToMeters(this.rightLeader.getSelectedSensorPosition());
	}

	public double getAvgDistanceMeters() {
		return (this.getLeftDistanceMeters() + this.getRightDistanceMeters()) / 2;
	}

	public double[] readGyro() {
		double[] angle = new double[3];
		this.pigeon.getYawPitchRoll(angle);
		return angle;
	}

	/**
	 * 
	 * @return pose from encoders
	 */
	public Pose2d getEncoderPose() {
		return this.odometry.getPoseMeters();
	}

	/**
	 * If the robot is in simulation then a default pose is returned
	 * 
	 * @return pose using encoders and limelight
	 */
	public Pose2d getEstimatedPose(){
		if(RobotBase.isReal()) {
			return this.poseEstimator.getEstimatedPosition();
		} else if(Timer.getFPGATimestamp() > 0.5) {
			return new Pose2d(5.0,4.0, new Rotation2d(3.1));
		} 
		else {
			return new Pose2d(0.0,0.0, new Rotation2d());
		}				
	}

	public Rotation2d readYawRot() {
		return Rotation2d.fromDegrees(this.readYaw());
	}

	public double readYaw() {
		return this.readGyro()[0];
	}

	public double readPitch() {
		return -this.readGyro()[1];
	}

	public double readRoll() {
		return -this.readGyro()[2];
	}

	public Rotation2d read2dRotation() {
		return Rotation2d.fromDegrees(this.readYaw());
	}

	public Pose2d getPose() {
		return this.odometry.getPoseMeters();
	}

	public double getMotorOutput() {
		return this.rightLeader.getMotorOutputVoltage();
	}

	public double metersToWheelRotations(double metersPerSecond) {
		return metersPerSecond / (DrivetrainConstants.kWheelDiameterMeters * Math.PI);
	}

	public double wheelRotationsToEncoderTicks(double wheelRotations, Transmission.GearState gearState) {
		if(gearState == Transmission.GearState.HIGH) {
			return wheelRotations * DrivetrainConstants.encoderCPR * DrivetrainConstants.highGearRatio;
		}
		return wheelRotations * DrivetrainConstants.encoderCPR * DrivetrainConstants.lowGearRatio;
	}

	public double metersToEncoderTicks(double metersPerSecond) {
		GearState gearState = this.gearStateSupplier.get();
		double encoderTicks = this.wheelRotationsToEncoderTicks(
			this.metersToWheelRotations(metersPerSecond),
			gearState
		);
		return encoderTicks;
	}	

	public double getHeading() {
		return this.pigeon.getYaw();
	}

	// calculations are in meters
	public Pose2d getLimelightPose(){
		// Rotation2d rotation = new Rotation2d(this.limelight.getPose()[5] / 180 * Math.PI);
		Rotation2d rotation = Rotation2d.fromDegrees(this.readYaw());
		return new Pose2d(
			this.limelight.getPose()[0] + DrivetrainConstants.xOffsetField, 
			this.limelight.getPose()[1] + DrivetrainConstants.yOffsetField,
			rotation
		);
	}

  	/** 
	 * Returns if current robot estimated pose is left or right of the center
	 * of the Charging Station taking the team alliance into account.
	 * 
	 * @return Is robot left or right of the center of the Charging Station
	 */
	public boolean isLeftOfChargingStation() {
		if(RobotContainer.alliance == DriverStation.Alliance.Red){
			return this.getEstimatedPose().getY() >= FieldConstants.Community.chargingStationCenterY;
		} else {
			return this.getEstimatedPose().getY() <= FieldConstants.Community.chargingStationCenterY;
		}		
	}

	public boolean isRightOfChargingStation() {
		return !this.isLeftOfChargingStation();
	}

	/**
	 * 
	 * @param startPose pose where robot starts
	 * @param endPose pose where robot should end
	 * @param direction 0 for left, 1 for right, 2 for robot to decide
	 * @return
	 */
	public Trajectory navigateToDropoff(Pose2d endPose, int direction){
		Trajectory trajectory;
		Pose2d startPose;

		Log.writeln("End pose: " + endPose);
		
		startPose = this.getEstimatedPose();

		DriverStation.Alliance color = DriverStation.getAlliance();

		// trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of (new Translation2d(1, 0)),
		// 									new Pose2d(2, 0, new Rotation2d(0)), DrivetrainConstants.kTrajectoryConfig);
		
		if(color == DriverStation.Alliance.Red){
			// for red, left and right
			//if direction is specified left, or direction is unspecified and Y is on left side of field...
			if(direction == 0 || ((direction == 2 ) && (startPose.getY() <= (DrivetrainConstants.fieldWidthYMeters / 2)))){
				trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
				//List.of(DrivetrainConstants.leftRedWaypoint1, DrivetrainConstants.leftRedWaypoint2), \
				List.of(),
				endPose, DrivetrainConstants.kTrajectoryConfig);
			} else {
				trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
					//List.of(DrivetrainConstants.rightRedWaypoint1, DrivetrainConstants.rightRedWaypoint2), 
					List.of(),
					endPose, DrivetrainConstants.kTrajectoryConfig);
			}
		} else {
			// for blue, left and right
			if(direction == 0 || ((direction == 2 ) && (startPose.getY() >= (DrivetrainConstants.fieldWidthYMeters / 2)))){
				trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
					//List.of(DrivetrainConstants.leftBlueWaypoint1, DrivetrainConstants.leftBlueWaypoint2), 
					List.of(),
					endPose, DrivetrainConstants.kTrajectoryConfig);
			} else {
				trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
					//List.of(DrivetrainConstants.rightBlueWaypoint1, DrivetrainConstants.rightBlueWaypoint2), 
					List.of(),
					endPose, DrivetrainConstants.kTrajectoryConfig);
			}
		}


		return trajectory;
	}

	// ----------------------------------------------------
	// Process Logic
	// ----------------------------------------------------

	@Override
	public void periodic() {
		//if limelight sees april tags, use limelight odometry, otherwise update from pigeon and encoders
		// if(limelight.getHasValidTargets() == 1){
		// 	updateOdometryFromLimelight();
		// } else {
		// 	  odometry.update(readYawRot(), getLeftDistanceMeters(), getRightDistanceMeters());
		// }

		this.odometry.update(this.readYawRot(), this.getLeftDistanceMeters(), this.getRightDistanceMeters());
		this.poseEstimator.update(this.readYawRot(), this.getLeftDistanceMeters(), this.getRightDistanceMeters());
		if(this.limelight.getHasValidTargets() == 1) {
			poseEstimator.addVisionMeasurement(this.getLimelightPose(), Timer.getFPGATimestamp() - 0.3);
		}

		this.publishTelemetry();
	}

	public void publishTelemetry() {
		SmartDashboard.putNumber("Odometry X", this.odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("Odometry Y", this.odometry.getPoseMeters().getY());
		SmartDashboard.putNumber("Odometry Theta", this.odometry.getPoseMeters().getRotation().getDegrees());
		SmartDashboard.putNumber("left encoder", this.leftLeader.getSelectedSensorPosition());
		SmartDashboard.putNumber("right encoder", this.rightLeader.getSelectedSensorPosition());
		SmartDashboard.putNumber("limelight X", this.limelight.getPose()[0]);
		this.field2d.setRobotPose(this.getEncoderPose());
		this.fieldEstimated.setRobotPose(this.getEstimatedPose());
		this.fieldLimelight.setRobotPose(this.getLimelightPose());

		// SmartDashboard.putNumber("motor output", this.getMotorOutput());	
		// SmartDashboard.putNumber("right enoder ticks", this.rightLeader.getSelectedSensorPosition());
		// SmartDashboard.putNumber("left enoder ticks", this.leftLeader.getSelectedSensorPosition());
		// SmartDashboard.putNumber("poseX", this.getEncoderPose().getX());
		// SmartDashboard.putNumber("botposeX", (this.limelight.getPose()[0] - DrivetrainConstants.xOffsetField));
	}
	
	public Trajectory generateTrajectory(Pose2d endPose) {
        Pose2d startPose = this.getEstimatedPose();
        
		Log.writeln("Initial Pose: " + startPose.getX());

        SmartDashboard.putNumber("Start Pose X", startPose.getX());
        SmartDashboard.putNumber("Start Pose Y", startPose.getY());
        SmartDashboard.putNumber("Start Pose Theta", startPose.getRotation().getDegrees());
    
        SmartDashboard.putNumber("End Pose X", endPose.getX());
        SmartDashboard.putNumber("End Pose Y", endPose.getY());
        SmartDashboard.putNumber("End Pose Theta", endPose.getRotation().getDegrees());
    
        DriverStation.Alliance color = DriverStation.getAlliance();
        
        // if(color == DriverStation.Alliance.Red){
        // 	// for red, left and right
        // 	//if direction is specified left, or direction is unspecified and Y is on left side of field...
        // 	if(direction == 0 || ((direction == 2 ) && (startPose.getY() <= (DrivetrainConstants.fieldWidthYMeters / 2)))){
        // 		RobotContainer.dynamicTrajectory = TrajectoryGenerator.generateTrajectory(startPose, 
        // 		//List.of(DrivetrainConstants.leftRedWaypoint1, DrivetrainConstants.leftRedWaypoint2), \
        // 		List.of(),
        // 		endPose, DrivetrainConstants.kTrajectoryConfig);
        // 	} else {
        // 		RobotContainer.dynamicTrajectory = TrajectoryGenerator.generateTrajectory(startPose, 
        // 			//List.of(DrivetrainConstants.rightRedWaypoint1, DrivetrainConstants.rightRedWaypoint2), 
        // 			List.of(),
        // 			endPose, DrivetrainConstants.kTrajectoryConfig);
        // 	}
        // } else {
        // 	// for blue, left and right
        // 	if(direction == 0 || ((direction == 2 ) && (startPose.getY() >= (DrivetrainConstants.fieldWidthYMeters / 2)))){
        // 		RobotContainer.dynamicTrajectory = TrajectoryGenerator.generateTrajectory(startPose, 
        // 			//List.of(DrivetrainConstants.leftBlueWaypoint1, DrivetrainConstants.leftBlueWaypoint2), 
        // 			List.of(),
        // 			endPose, DrivetrainConstants.kTrajectoryConfig);
        // 	} else {
        // 		RobotContainer.dynamicTrajectory = TrajectoryGenerator.generateTrajectory(startPose, 
        // 			//List.of(DrivetrainConstants.rightBlueWaypoint1, DrivetrainConstants.rightBlueWaypoint2), 
        // 			List.of(),
        // 			endPose, DrivetrainConstants.kTrajectoryConfig);
        // 	}
        // }
    
        SmartDashboard.putNumber("Waypoint1 X", FieldConstants.Waypoints.rightBlue1.getX());
        SmartDashboard.putNumber("Waypoint Y", FieldConstants.Waypoints.rightBlue1.getY());
    
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
                List.of(FieldConstants.Waypoints.rightBlue1),
                // List.of(),
                endPose, DrivetrainConstants.kTrajectoryConfig);
    
        Log.writeln("Traj: " + trajectory.getTotalTimeSeconds()); 

        return trajectory;
    }

	// ----------------------------------------------------
	// Simulation
	// ----------------------------------------------------

	@Override
    public void simulationPeriodic() {
        this.driveSim.run();
    }
}
