// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


public final class Constants{

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public static final class DriveConstants {

  public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics
        (
            new Translation2d(kWheelBase / 2, kTrackWidth /2),
            new Translation2d(kWheelBase / 2, -kTrackWidth /2),
            new Translation2d(-kWheelBase / 2, kTrackWidth /2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth /2)
        );
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI /2;

        public static final int kFrontLeftDrivingCanId =  7;
        public static final int kRearLeftDrivingCanId = 8;
        public static final int kFrontRightDrivingCanId = 4;
        public static final int kRearRightDrivingCanId = 3;

        public static final int kFrontLeftTurningCanId =  6;
        public static final int kRearLeftTurningCanId = 9;
        public static final int kFrontRightTurningCanId = 5;
        public static final int kRearRightTurningCanId = 2;
        
        public static final Boolean kGyroReversed = false;
}
public static final class ModuleConstants
{
    public static final double kDrivingMotorPinionTeeth = 14;

    public static final double kDrivingMoterFreeSpeedRps = NeoMotorConsts.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferencMeters = kWheelDiameterMeters * Math.PI;

    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth *15 );
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMoterFreeSpeedRps * kWheelCircumferencMeters) / kDrivingMotorReduction;
}

public static final class OIConstants
{
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;//Deadband?
    public static final double kTriggerbuttonThreshold = 0.2;
}

public static final class AutoConstants
{
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAcclerationMetersPerSecondsSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints
        (
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
        );  
}

public static final class NeoMotorConsts
{
    public static final double kFreeSpeedRpm = 5676;
}
public static final class SimlationRobotConstants
{
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 25;
    public static final double kCarriageMass = 4.3 + 3.15 + 0.151;
    public static final double kElevatorDrumRadiun = 0.0328 / 2.0;
    public static final double kMinkElevatorDrumRadiun = 0.922;
    public static final double kMaxkElevatorDrumRadiun = 1.62;
}
}
