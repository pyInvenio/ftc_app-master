package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import java.util.Arrays;
import java.util.List;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
/**
 * Created by infoadv on 1/15/2017.
 */
@Autonomous(name="DistanceTest", group="Blue")
@Disabled


/**
 * Created by infoadv on 11/11/2017.
 */

public class DistanceTest extends Shockwave71712017CommonLinearOp {

    /* Declare OpMode members. */
    Shockwave71712017Hardware robot = new Shockwave71712017Hardware();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    static final double WHITE_THRESHOLD = 0.5;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.5;
    float hsvValues[] = {0F, 0F, 0F};
    int gameStartAngle;
    int currentAngle;
    double servoPosition;
    List<Integer> list = Arrays.asList(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, -179, -178, -177, -176, -175, -174, -173, -172, -171, -170, -169, -168, -167, -166, -165, -164, -163, -162, -161, -160, -159, -158, -157, -156, -155, -154, -153, -152, -151, -150, -149, -148, -147, -146, -145, -144, -143, -142, -141, -140, -139, -138, -137, -136, -135, -134, -133, -132, -131, -130, -129, -128, -127, -126, -125, -124, -123, -122, -121, -120, -119, -118, -117, -116, -115, -114, -113, -112, -111, -110, -109, -108, -107, -106, -105, -104, -103, -102, -101, -100, -99, -98, -97, -96, -95, -94, -93, -92, -91, -90, -89, -88, -87, -86, -85, -84, -83, -82, -81, -80, -79, -78, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60, -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1);
    int team_color;
    int redValue;
    int blueValue;
    int a;
    double start;
    int count =0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
//        robot.v_servo_push_one.setPosition(.45);
//        robot.v_servo_push_two.setPosition(0);
//        robot.v_servo_jewel.setPosition(0.85);//0.45
        servoPosition = robot.v_servo_jewel.getPosition();
        telemetry.addData("ServoPosition", servoPosition);
        telemetry.update();
        //for move 5 times the servo to detec red or blue
        // break from for loop if red or blue is non zero
        a = 0;
        start = System.currentTimeMillis();
        double x = robot.v_range_sensor.getDistance(DistanceUnit.CM);
       /* if(vuMark == left)
        {
            int y = 2;
        }
        else if(vuMark == center)
        {
            int y = 3;
        }
        else if(vumark == right)
        {
            int y = 4;
        }
        for(int i =0; i<= y; i++) {*/
/*        while(true)
       */
        {
            telemetry.addData("Distance", robot.v_range_sensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            // }*/
            sleep(1000);
            telemetry.addData("Before", "While");
            telemetry.update();
            goForwardNRotations(robot, 500, -.3);
            for(int i = 0; i < 2; i++) {
                while (robot.v_range_sensor.getDistance(DistanceUnit.CM) >= x - 4)

                {
                    x = robot.v_range_sensor.getDistance(DistanceUnit.CM);

                    telemetry.addData("Distance", robot.v_range_sensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                    goForwardNRotations(robot, 10, -0.3);
                    sleep(500);

                }
                count++;
            }
            turnLeft(robot, 200, -0.2);
            telemetry.addData("Count:", count);
            telemetry.update();
            sleep(2000);

//    }
//        robot.v_motor_lift.setPower(.5);
//        while (true) {
//            if (opModeIsActive() == false)
//                break;
////            robot.v_motor_lift.setPower(-.5);
//            if (System.currentTimeMillis() - start >= 1000)
//                break;
//        }
//        robot.v_motor_lift.setPower(0);
//        for (int k = 1; k <= 5; k++) {
//            if (opModeIsActive() == false)
//                break;
//            redValue = robot.v_jewel_color_sensor.red();
//            blueValue = robot.v_jewel_color_sensor.blue();
//
//
//            if (redValue != 0 || blueValue != 0)
//                break;
//            else {
//                servoPosition = servoPosition + 0.03;
//                robot.v_servo_jewel.setPosition(servoPosition);
//                a++;
//                if (a % 3 == 0) {
////                    goForwardNRotations(robot, 10, .1);
//                    robot.v_servo_jewel.setPosition(robot.v_servo_jewel_horizontal.getPosition() + .03);
//                }
//                sleep(2000);
//            }
//        }
//
//        /*
//        if color detects blue
//            servo 2 moves to the left
//        else
//            servo 2 moves to the right
//        */
//        double x = robot.v_servo_jewel_horizontal.getPosition();
//        telemetry.addData("Servo Postion", x);
//        telemetry.update();
//        sleep(2000);
//
//        if (blueValue != 0 && opModeIsActive() == true) {
//        robot.v_servo_jewel_horizontal.setPosition(0.8);
//            telemetry.addData("Blue:", blueValue);
//            telemetry.update();
//            telemetry.addData("Red", redValue);
//            telemetry.update();
//            sleep(5000);
//        }
//        else if(redValue != 0 && opModeIsActive() == true){
//            robot.v_servo_jewel_horizontal .setPosition(0.2);
//            telemetry.addData("Blue:", blueValue);
//            telemetry.update();
//            telemetry.addData("Red", redValue);
//            telemetry.update();
//            sleep(5000);
//        }
//        else{
//            telemetry.addData("None Detected","!");
//            telemetry.update();
//        }
//        robot.v_servo_jewel.setPosition(0.0);
//        robot.v_servo_jewel_horizontal.setPosition(0.6);
//        sleep(5000);

//        if (blueValue != 0 && opModeIsActive() == true) {
////            goForwardNRotations(robot, 150, -0.2);
////            //robot.v_servo_jewel.setPosition(0.0);
////            sleep(1000);
////            //goForwardNRotations( robot, 150, 0.2);
////            // move robot front
//            robot.v_servo_jewel_horizontal.setPosition(0.0)
//
//            telemetry.addData("Blue Found", blueValue);
//        } else {
//            goForwardNRotations(robot, 150, 0.2);
//            robot.v_servo_jewel.setPosition(0.0);
//            sleep(1000);
//            goForwardNRotations(robot, 300, -0.2);
//            //move robot back
//            telemetry.addData("Red Found", redValue);
//        }
        } // end if(blueValue != 0)
    }
}

