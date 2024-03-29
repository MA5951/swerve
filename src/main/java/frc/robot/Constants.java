// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class FieldConstants {
        public static final double FIELD_WIDTH_METERS = 8.02;
        public static final double FIELD_LENGTH_METERS = 16.54;
    }

    public static class Camera {
        public static final double CAMERA_DISTANCE_FROM_CENTER_IN_X = 0.1;
        public static final double CAMERA_DISTANCE_FROM_CENTER_IN_Y = 0.2;
        public static final double CAMERA_DISTANCE_FROM_CENTER_IN_Z = 0.7476;
        public static final double CAMERA_ROLL = 0;
        public static final double CAMERA_PITCH = 0;
        public static final double CAMERA_YAW = 0;
    }
}
