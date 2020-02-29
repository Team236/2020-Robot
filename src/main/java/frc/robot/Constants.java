/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import lib.motionProfile.DriveParameters;
import lib.pid.PIDParameters;

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

    public static class ControllerConstants {
        public static final int USB_LEFT_STICK = 0;
        public static final int USB_RIGHT_STICK = 1;
        public static final int USB_CONTROLLER = 2;
    }

    public static class DriveConstants {
        public static final int ID_LEFT_FRONT = 7;
        public static final int ID_LEFT_REAR = 8;
        public static final int ID_RIGHT_FRONT = 4;
        public static final int ID_RIGHT_REAR = 6;

        public static final double DIAMETER = 6.0;
        public static final double CIRCUMFERENCE = Math.PI * DIAMETER;
        // public static final double GEAR_RATIO = (46/11) * (52/20);
        public static final double GEAR_RATIO = 8.71;
        // constant by which to multiply to convert revolutions to in
        // public static final double REV_TO_IN_K = GEAR_RATIO / CIRCUMFERENCE;
        public static final double REV_TO_IN_K = CIRCUMFERENCE / GEAR_RATIO;
        public static final double IN_TO_REV_K = GEAR_RATIO / CIRCUMFERENCE;

        public static final double LEFT_DEADZONE = .1;
        public static final double RIGHT_DEADZONE = .1;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;

        public static final double MIN_OUTPUT = 0.0;
        public static final double MAX_OUTPUT = 1.0;

        public static final boolean IS_DEADZONE = true;

        public static final DriveParameters DRIVE_PARAMS = new DriveParameters(0.0, 0.0, 0.0, 0.0, -0.0);
    }

    public static class AutoConstants {
        public static final int DIO_SWITCH_1 = 6;
        public static final int DIO_SWITCH_2 = 7;
        public static final int DIO_SWITCH_3 = 8;
        public static final int DIO_SWITCH_4 = 9;

        // TURN CONSTANTS
        public static final PIDParameters TURN_PARAMS = new PIDParameters(0.0075, 0.0, 0.0, .02);
    }

    public static class IntakeConstants {
        public static final int ID_MOTOR = 24;
        public static final int ID_POSITION_MOTOR = 99;
        public static final int DIO_INTAKE_COUNTER = 0;
        public static final int DIO_UPPER_LIMIT = 13;
        public static final int DIO_LOWER_LIMIT = 13;

        public static final double LIME_KP = .005;
        public static final double LIME_KI = .0;
        public static final double LIME_KD = .005;
        public static final double LIME_SPEED = 0.2;

        public static final double SPEED = 0.60;
        public static final double RAISE_SPEED = 1.0;
        public static final double LOWER_SPEED = -RAISE_SPEED;

        public static final boolean CONSIDER_COUNT = false;
        public static final int MAX_COUNT = 6;

        public static final double I_ACTIVE_ZONE = 3.00;
    }

    public static class ShooterConstants {
        public static final int ID_MASTER = 13;
        public static final int ID_FOLLOWER = 9;
        public static final int ID_HOOD = 8;

        public static final int DIO_SHOOT_COUNTER = 5;
        public static final int DIO_HOOD_LIMIT = 99;

        public static final double SPEED_RPM = 500;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.0002;

        public static final double HOOD_kP = 0.8;
        public static final double HOOD_kI = 0.0;
        public static final double HOOD_kD = 0.0;
        public static final double HOOD_ACTIVE_ZONE = 5.0;

        public static final double HOOD_SPEED = .8;

        public static final double MIN_OUTPUT = 0.0;
        public static final double MAX_OUTPUT = 6000.0;
        public static final double interval = 0.0;

    }

    public static class ColorSpinnerConstants {
        public static final int ID_MOTOR = 14;
        public static final int PWM_SERVO = 0;

        public static final double SPEED = 0.05;

        public static final int COLOR_COUNT = 32;

        // Exten/retract servo position values
        public static final double EXTEND_VALUE = 1.0; // .8
        public static final double RETRACT_VALUE = 0.0; // .2

        // Extend/retract servo run-times
        public static final double EXTEND_TIME = 1.4;
        public static final double RETRACT_TIME = 0.6;
    }

    public static class CarouselConstants {
        public static final int ID_MOTOR = 25;
        public static final int ID_ROLLER_MOTOR = 99;
        public static final int ID_GREEN_WHEEL = 99;

        public static final int PWM_TO_SHOOT_SERVO = 2;
        public static final int PWM_TO_SHOOT_SERVO_2 = 3;

        public static final double SPEED = .5;
        public static final double GREEN_SPEED = .5;
        public static final double ROLLER_SPEED = .5;

        public static final double EXTEND_POS = 1.0;
        public static final double RETRACT_POS = 0.0;
    }

    public static class TurretConstants {
        public static final int ID_TURRET = 12;
        // public static final int DIO_TURRET = 0;
        public static final int DIO_LEFT_LIMIT = 4;
        public static final int DIO_RIGHT_LIMIT = 5;

        public static final double TURRET_kP = 0.03;
        public static final double TURRET_kI = 0.0045;
        public static final double TURRET_kD = 0.01;
        public static final double TURRET_SPEED = 0.4;
        public static final double I_ACTIVE_ZONE = 5.00;
    }

    public static class ClimberConstants {
        public static final int ID_MASTER = 0;
        public static final int ID_FOLLOWER = 0;

        public static final int DIO_TOP_LIMIT = 11;
        public static final int DIO_BOT_LIMIT = 11;

        public static final double SPEED = 1.0;
        public static final double ENC_LIMIT = 1000;
    }
}
