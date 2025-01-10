// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.util.ModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class SwerveConstants {

    public static final IdleMode driveIdleMode = IdleMode.kBrake;
    public static final IdleMode angleIdleMode = IdleMode.kBrake;
    public static final double drivePower = 1;
    public static final double anglePower = .9;

    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    // drivetrain constants
    public static final double trackWidth = Units.inchesToMeters(23.75); 
    public static final double wheelBase = Units.inchesToMeters(23.75); 
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    // Swerve kinematics, don't change
     public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    // gear ratios
    public static final double driveGearRatio = (6.12 / 1.0);
    public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);

    // encoder stuff
    // meters per rotation
    public static final double driveRevToMeters =  wheelCircumference / (driveGearRatio);
    public static final double driveRpmToMetersPerSecond = driveRevToMeters / 60 ;
    // the number of degrees that a single rotation of the turn motor turns the wheel.
    public static final double DegreesPerTurnRotation = 360 / angleGearRatio;

    
    // motor inverts, check these
    public static final boolean angleMotorInvert = true;
    public static final boolean driveMotorInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveSupplyCurrentLimit = 60;
    public static final boolean driveSupplyCurrentLimitEnable = true;
    public static final int driveSupplyCurrentThreshold = 60;
    public static final double driveSupplyTimeThreshold = 0.1;
    
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = 0;
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double angleKF = 0;

    /* Drive Motor PID Values */

    public static final double driveKP = 0.1; 
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /* Heading PID Values */
    public static final double headingKP = 4;
    public static final double headingKI = 0.0;
    public static final double headingKD = 0;
    public static final double headingTolerence = 0;


    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12); 
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.0;
    /** Radians per Second */
    public static final double maxAngularVelocity = 5.0;
    
    public static final class Mod0 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

        /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
        
        /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

        /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
