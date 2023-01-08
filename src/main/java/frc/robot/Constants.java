// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class PneumaticIDs {
    //Solenoids
   //  public static final int kDrivetrainShiftSolenoid = 0;
    public static final int kRampSolenoid = 1;
    public static final int kClimberSolenoid = 2;

    public static final int kDrivetrainShiftSolenoidLow = 2;
    public static final int kDrivetrainShiftSolenoidHigh = 3;
    // public static final int kRampSolenoidOpen = 2;
    // public static final int kRampSolenoidClosed = 3;

  
}
public static final class CANBusIDs {
    // Drivetrain, right side
    public static final int kDrivetrainRightBackTalonFX = 0;
    public static final int kDrivetrainRightFrontTalonFX = 1;
   // Drivetrain, left side
   public static final int kDrivetrainLeftFrontTalonFX = 14;
   public static final int kDrivetrainLeftBackTalonFX = 15;   
   
   //Sensors
   public static final int kPigeonIMU = 3;
   }  

   public static final class DrivetrainConstants{

    // kS (static friction), kV (velocity), and kA (acceleration)
    public static final double ksVolts = 0.6024;
    public static final double kvVoltSecondsPerMeter = 0.21907;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0096252;

    // Feedforward contraints          
   public static final SimpleMotorFeedforward kFeedForward = 
   new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

    public static final double kTrackWidthMeters = 0.7; //Placeholder
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final double k_MaxVolts = 10;    
    public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            kFeedForward,
            kDriveKinematics,
            k_MaxVolts);    

    public static final boolean kGyroReversed = true;

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1015;

    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kUnitsPerRevolution = 2048;

    //TODO change to correct values
    public static final double kHighGearRatio = 10.71;
    public static final double kLowGearRatio = 5;

    public static final double kMaxSpeedMetersPerSecond = 2.0;
    public static final double kMaxAccelMetersPerSecondSquared = 2.0;

    public static final TrapezoidProfile.Constraints kTrapezoidProfileConstraints =
        new TrapezoidProfile.Constraints(kMaxSpeedMetersPerSecond, kMaxAccelMetersPerSecondSquared);
    
    // PID Constants
    // The WPILib feedforward has kS (static friction), kV (velocity), and kA (acceleration) terms 
    // whereas the Talon SRX / Spark MAX kF is only a kV (velocity) feedforward.
    //                                                  kp,  ki, kd,  kf, iz,  peak output
    public static final Gains kGainsProfiled = new Gains(0.16,  0,   0,   0,   0,  1.00);   

    /**
     * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
     * Not all set of Gains are used in this project and may be removed as desired.
     * 
     * 	                                    			  kP   kI   kD   kF               Iz    PeakOut */
    public final static Gains kGainsDistance = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );
    public final static Gains kGainsTurning = new Gains( 0.10, 0.0,  0.0, 0.0,            200,  1.00 );
    public final static Gains kGainsVelocity = new Gains( 0.1, 0.001, 5, 1023.0/20660.0,  300,  1.00);
    public final static Gains kGainsMotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/20660.0,  400,  1.00 );

    //public static final double kDistanceToleranceMeters = 0.1;
    //public static final double kVelocityToleranceMeters = 0.1;
}

}
