/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class Gains {
	// Proportional gain for closed loop. This is multiplied by closed loop error in sensor units. 
	// Note the closed loop output interprets a final value of 1023 as full output. 
	// So use a gain of ‘0.25’ to get full output if err is 4096u (Mag Encoder 1 rotation)
	public final double kP;

	// Integral gain for closed loop. 
	// This is multiplied by closed loop error in sensor units every PID Loop. 
	public final double kI;

	// Derivative gain for closed loop. 
	// This is multiplied by derivative error (sensor units per PID loop). 
	public final double kD;

	// Feed Forward gain for Closed loop. 
	// If using velocity, motion magic, or motion profile, use (1023 * duty-cycle / sensor-velocity-sensor-units-per-100ms)
	public final double kF;
	
	// Integral Zone can be used to auto clear the integral accumulator if the sensor position
	// is too far from the target. This prevent unstable oscillation if the kI is too large. 
	// Value is in sensor units.
	public final int kIzone;

	// Absolute max motor output during closed-loop control modes only. 
	// A value of ‘1’ represents full output in both directions.
	public final double kPeakOutput;
	
	public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
	}
}
