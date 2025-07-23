/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class Steering{

        public static final int k_FLZero = 1268;
        public static final int k_FRZero = -2064;
        public static final int k_BLZero = 218;
        public static final int k_BRZero = -946;

        public static final double k_FLMinSteerPower = .2;
        public static final double k_FRMinSteerPower = .28;
        public static final double k_BLMinSteerPower = .28;
        public static final double k_BRMinSteerPower = .28;

        public static final double k_frontLeftSteeringP =  0.0028; //.8 / 360; 
        public static final double k_frontRightSteeringP = .045 / 360;
        public static final double k_backLeftSteeringP = .045 / 360;
        public static final double k_backRightSteeringP = .045 / 360;

        public static final double k_frontLeftSteeringD = 0.005;
        public static final double k_frontRightSteeringD = 0.005;
        public static final double k_backLeftSteeringD = 0.005;
        public static final double k_backRightSteeringD = 0.005;
    }
    public static class Drive{
        public static final double k_maxSpeed = 2000;

        public static final double k_FLminDrivePower = 0.45;
        public static final double k_FRminDrivePower = 0.45;
        public static final double k_BLminDrivePower = 0.45;
        public static final double k_BRminDrivePower = 0.45;

        public static final double k_frontLeftDriveF = 1.0 /k_maxSpeed; 
        public static final double k_frontRightDriveF = 1.0 /k_maxSpeed;
        public static final double k_backLeftDriveF = 1.0 /k_maxSpeed;
        public static final double k_backRightDriveF = 1.0 /k_maxSpeed;

        public static final double k_DriveP = 0.001; 
        public static final double k_DriveITerm = 0.0001;
        public static final double k_DriveIZone = 200;

    }
}
