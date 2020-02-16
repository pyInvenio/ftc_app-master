/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

//import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * GRYRO ANGLES
 *          0
 *      90      270
 *          180
 *   USE TURN  LEFT if need to go to 90
 */

@TeleOp(name="Common2016", group="7171")
@Disabled
public class Shockwave71712017CommonLinearOp extends LinearOpMode {
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    double lightValue;
    int TURN_RIGHT = 1 ;
    int TURN_LEFT = 0 ;
    int RED = 1;
    int BLUE = 0;
    int startAngle;
    int position;
    int armDistance = 25;
//    int relicCorrectionMove = 0;
//    int SINITDIS2 = 55;
//    int PARTITIONDISTANCE = 19;
//    int NSINITDIS = 120;
//    int SINITDIS1 = 40;
//int AUTOLIFTTIME = 1200 ;
//    double AUTOLIFTMOTORPOWER = 0.75;
//    int backUpCount = 0;
//    double fExtendGearPower = 0.0 ;
//    int LEFT_EDGE = 1 ;
//    int RIGHT_EDGE = 2 ;
//    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
//    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.
//
//    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
//    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
//    static final double     P_DRIVE_COEFF           = 0.15;
//int RED_TEAM = 1;
//    int BLUE_TEAM = 2 ;
//    int DIR_LEFT = 1 ;
//    int DIR_RIGHT = 2 ;

    List<Integer> list = Arrays.asList(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, -179, -178, -177, -176, -175, -174, -173, -172, -171, -170, -169, -168, -167, -166, -165, -164, -163, -162, -161, -160, -159, -158, -157, -156, -155, -154, -153, -152, -151, -150, -149, -148, -147, -146, -145, -144, -143, -142, -141, -140, -139, -138, -137, -136, -135, -134, -133, -132, -131, -130, -129, -128, -127, -126, -125, -124, -123, -122, -121, -120, -119, -118, -117, -116, -115, -114, -113, -112, -111, -110, -109, -108, -107, -106, -105, -104, -103, -102, -101, -100, -99, -98, -97, -96, -95, -94, -93, -92, -91, -90, -89, -88, -87, -86, -85, -84, -83, -82, -81, -80, -79, -78, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60, -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1);
    /*
     * Turns Robot right
       */

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
//
//    /*
//     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
//     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
//     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
//     * web site at https://developer.vuforia.com/license-manager.
//     *
//     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
//     * random data. As an example, here is a example of a fragment of a valid key:
//     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
//     * Once you've obtained a license key, copy the string from the Vuforia web site
//     * and paste it in to your code on the next line, between the double quotes.
//     */
    private static final String JXP_VUFORIA_KEY =
            "AVcokEn/////AAAAGYxCoQ/keUsCglbU/LG4i8c/L2+TX9r70WyGPbO2JSTjA6hUGU5WZiYVyZRBFXi09YrcYeairjLBq1DGwFV9ZgDZWNb5+e5xm7oRimAQBladvj6vwzHbaP3TKemPgFd9ANPhxptpUKuOE/y4P2SXKnQUNiJZjFVvC2NV3OxSc3FzdQ45nsqJkgH0/dY1nuBbYM50uMy4fzsUEUHmSLyrLsQS8vXOdFDdcBcc4TBEiqd2aHbGHafpM1wz2KRzmN3XgDDBC2u14fkNsM4vmP1qwFPvda+3LaC6U/i+6obHQ2tG/CKXjq84IZnudFYLbmClhuacZaGbi8yNkiZJwwzPm7Bv0Ocd9B2560qJNea8B9gw";
//
//    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
//    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
//    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
//
//    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;
//
//    // Constants for the center support targets
//    private static final float bridgeZ = 6.42f * mmPerInch;
//    private static final float bridgeY = 23 * mmPerInch;
//    private static final float bridgeX = 5.18f * mmPerInch;
//    private static final float bridgeRotY = 59;                                 // Units are degrees
//    private static final float bridgeRotZ = 180;
//
//    // Constants for perimeter targets
//    private static final float halfField = 72 * mmPerInch;
//    private static final float quadField  = 36 * mmPerInch;
//
//    // Class Members
    private OpenGLMatrix JXPlastLocation = null;
    public VuforiaLocalizer JXPvuforia = null;
    public boolean JXPtargetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
//
//
    final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
    VuforiaLocalizer.Parameters JXPparameters;
    OpenGLMatrix JXProbotFromCamera;
  public int JXPcameraMonitorViewId ;
    VuforiaTrackables JXPtargetsSkyStone ;

    private boolean JXPycoordDone = false;
    private boolean JXPxcoordDone = false;
    private double lastAngle = 0;
    private double globalAngle = 0;

    @Override
    public void runOpMode() {
    }

    public float getX() {
        return phoneXRotate;
    }

    public void setX(float x) {
        phoneXRotate = x;
    }

    public float getY() {
        return phoneYRotate;
    }

    public void setY(float y) {
        phoneYRotate = y;
    }

    public float getZ() {
        return phoneZRotate;
    }

    public void setZ(float z) {
        phoneZRotate = z;
    }


    public boolean stopRobot(Shockwave71712017Hardware robot) {
        robot.v_motor_left_drive.setPower(0);
        robot.v_motor_right_drive.setPower(0);
        robot.v_motor_left_drive_back.setPower(0);
        robot.v_motor_right_drive_back.setPower(0);
        return true;
    }

    public boolean turnRight(Shockwave71712017Hardware robot, double numOfRotations, double power) {
        reset_drive_encoders(robot);
        run_using_encoders(robot);
        while (isEncoderValueReached(numOfRotations, robot.v_motor_right_drive) == false && opModeIsActive() == true) {
            robot.v_motor_left_drive.setPower(power);
            robot.v_motor_right_drive.setPower(-power);
            robot.v_motor_left_drive_back.setPower(power);
            robot.v_motor_right_drive_back.setPower(-power);
        }
        robot.v_motor_left_drive.setPower(0);
        robot.v_motor_right_drive.setPower(0);
        robot.v_motor_left_drive_back.setPower(0);
        robot.v_motor_right_drive_back.setPower(0);
        reset_drive_encoders(robot);
        telemetry.addData("Turn Right", "Return true");
        return true;
    }
    public boolean goForwardTargetRotations(Shockwave71712017Hardware robot, int numOfRotations, double power)
    {
        robot.v_motor_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.v_motor_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.v_motor_left_drive.setTargetPosition(numOfRotations);
        robot.v_motor_left_drive.setPower(power);
        robot.v_motor_right_drive.setPower(power);
        robot.v_motor_left_drive_back.setPower(power);
        robot.v_motor_right_drive_back.setPower(power);
        while(robot.v_motor_left_drive.isBusy() && opModeIsActive()){}
        robot.v_motor_left_drive.setPower(0);
        robot.v_motor_right_drive.setPower(0);
        robot.v_motor_left_drive_back.setPower(0);
        robot.v_motor_right_drive_back.setPower(0);
        return true;
    }
    public boolean goForwardNRotations (Shockwave71712017Hardware robot, double numOfRotations, double power) {
        double frontPower;
        reset_drive_encoders(robot);
        run_using_encoders(robot);
  //      sleep(1000);
        while ((isEncoderValueReached(numOfRotations, robot.v_motor_right_drive) == false) && (opModeIsActive() == true)) {
            robot.v_motor_left_drive.setPower(power);
            robot.v_motor_right_drive.setPower(power);
            robot.v_motor_left_drive_back.setPower(power);
            robot.v_motor_right_drive_back.setPower(power);
        }


        robot.v_motor_left_drive.setPower(0);
        robot.v_motor_right_drive.setPower(0);
        robot.v_motor_left_drive_back.setPower(0);
        robot.v_motor_right_drive_back.setPower(0);
        reset_drive_encoders(robot);
        return true;
    }

    /*
     * Turns Robot Left
       */
    public boolean turnLeft(Shockwave71712017Hardware robot, double numOfRotations, double power) {
        run_using_encoders(robot);
        while (isEncoderValueReached(numOfRotations, robot.v_motor_right_drive) == false && opModeIsActive() == true) {
            robot.v_motor_left_drive.setPower(-power);
            robot.v_motor_right_drive.setPower(power);
            robot.v_motor_left_drive_back.setPower(-power);
            robot.v_motor_right_drive_back.setPower(power);
        }
        robot.v_motor_left_drive.setPower(0);
        robot.v_motor_right_drive.setPower(0);
        robot.v_motor_left_drive_back.setPower(0);
        robot.v_motor_right_drive_back.setPower(0);
        reset_drive_encoders(robot);
        return true;
    }


    public void run_using_encoders(Shockwave71712017Hardware robot) {
        robot.v_motor_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.v_motor_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.v_motor_left_drive_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.v_motor_right_drive_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void run_without_encoders(Shockwave71712017Hardware robot) {
        robot.v_motor_right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.v_motor_left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.v_motor_left_drive_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.v_motor_right_drive_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void reset_drive_encoders(Shockwave71712017Hardware robot)

    {
        robot.v_motor_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.v_motor_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.v_motor_left_drive_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.v_motor_right_drive_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }



    public boolean isEncoderValueReached(double numOfRotations, DcMotor motor) {
       /* telemetry.addData("Encoder Values : ", motor.getCurrentPosition());
        telemetry.addData("numOfRotation : ", numOfRotations);
        telemetry.update () ;*/
        if (Math.abs(motor.getCurrentPosition()) >= numOfRotations) {
            return true;
        }
        return false;
    }

    public double getEncoderValue(DcMotor motor) {

        return (Math.abs(motor.getCurrentPosition()));
    }


    /*
     * Methods for GyroSensor
     */
    public boolean isGyroCalibrating(Shockwave71712017Hardware robot) {
        return robot.v_gyro_sensor.isCalibrating();
    }

    public boolean goForwardUsingGyro(Shockwave71712017Hardware robot, double numOfRotations, double power, int startangle) {
        run_using_encoders(robot);
        double PrepGain = 0.07;
        //  final double EncoderToInch = 71;
        List<Integer> list = Arrays.asList(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, -179, -178, -177, -176, -175, -174, -173, -172, -171, -170, -169, -168, -167, -166, -165, -164, -163, -162, -161, -160, -159, -158, -157, -156, -155, -154, -153, -152, -151, -150, -149, -148, -147, -146, -145, -144, -143, -142, -141, -140, -139, -138, -137, -136, -135, -134, -133, -132, -131, -130, -129, -128, -127, -126, -125, -124, -123, -122, -121, -120, -119, -118, -117, -116, -115, -114, -113, -112, -111, -110, -109, -108, -107, -106, -105, -104, -103, -102, -101, -100, -99, -98, -97, -96, -95, -94, -93, -92, -91, -90, -89, -88, -87, -86, -85, -84, -83, -82, -81, -80, -79, -78, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60, -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1);
        int testHeading ;
        double data ;
        double MotorCorrection ;

        while (isEncoderValueReached(numOfRotations, robot.v_motor_right_drive) == false && opModeIsActive()) {
            testHeading = list.get(robot.v_gyro_sensor.getHeading());
            data = testHeading - startangle;
            telemetry.addData("heading :", testHeading ) ;
            telemetry.addData("start angke:", startangle) ;
            telemetry.addData("data:", data) ;
            MotorCorrection = PrepGain * (data);
            robot.v_motor_left_drive.setPower(Range.clip(power + MotorCorrection, -1.0, 1.0));
            robot.v_motor_right_drive.setPower(Range.clip(power - MotorCorrection, -1.0, 1.0));
            robot.v_motor_left_drive_back.setPower(Range.clip((power + MotorCorrection), -1.0, 1.0));
            robot.v_motor_right_drive_back.setPower(Range.clip((power - MotorCorrection), -1.0, 1.0));
        }

        robot.v_motor_left_drive.setPower(0.0);
        robot.v_motor_right_drive.setPower(0.0);
        robot.v_motor_left_drive_back.setPower(0.0);
        robot.v_motor_right_drive_back.setPower(0.0);
        reset_drive_encoders(robot);
        return true;
    }

    public boolean goForwardUsingGyroNoRotation(Shockwave71712017Hardware robot, double power, int startangle) {
        double PrepGain = 0.07;
        //  final double EncoderToInch = 71;
        List<Integer> list = Arrays.asList(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, -179, -178, -177, -176, -175, -174, -173, -172, -171, -170, -169, -168, -167, -166, -165, -164, -163, -162, -161, -160, -159, -158, -157, -156, -155, -154, -153, -152, -151, -150, -149, -148, -147, -146, -145, -144, -143, -142, -141, -140, -139, -138, -137, -136, -135, -134, -133, -132, -131, -130, -129, -128, -127, -126, -125, -124, -123, -122, -121, -120, -119, -118, -117, -116, -115, -114, -113, -112, -111, -110, -109, -108, -107, -106, -105, -104, -103, -102, -101, -100, -99, -98, -97, -96, -95, -94, -93, -92, -91, -90, -89, -88, -87, -86, -85, -84, -83, -82, -81, -80, -79, -78, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60, -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1);
        int testHeading ;
        double data ;
        double MotorCorrection ;

        testHeading = list.get(robot.v_gyro_sensor.getHeading());
            data = testHeading - startangle;
            telemetry.addData("heading :", testHeading ) ;
            telemetry.addData("start angke:", startangle) ;
            telemetry.addData("data:", data) ;
            MotorCorrection = PrepGain * (data);
            robot.v_motor_left_drive.setPower(Range.clip(power + MotorCorrection, -1.0, 1.0));
            robot.v_motor_right_drive_back.setPower(Range.clip((power - MotorCorrection), -1.0, 1.0));
            robot.v_motor_right_drive.setPower(Range.clip(power - MotorCorrection, -1.0, 1.0));
            robot.v_motor_left_drive_back.setPower(Range.clip((power + MotorCorrection), -1.0, 1.0));
        return true;
    }


    public boolean turnGyroSpecialPivotToATargetAngle(Shockwave71712017Hardware robot, double targetAngle, int targetDirection, double multiplicationFactor, int startAngle ) {
        double turnPower;
        if (targetDirection == TURN_RIGHT)
            turnPower = -0.1*multiplicationFactor ;
        else
            turnPower = 0.1*multiplicationFactor ;

        boolean goalReached = false;
        reset_drive_encoders(robot);
        run_using_encoders(robot);

        while (goalReached == false && opModeIsActive()) {
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", robot.v_gyro_sensor.getHeading());
            telemetry.update();

            if (((robot.v_gyro_sensor.getHeading()) <= targetAngle + 1) && (robot.v_gyro_sensor.getHeading()) >= targetAngle - 1) {
//            if ((list.get(robot.v_gyro_sensor.getHeading()) <= targetAngle + 5) && (list.get(robot.v_gyro_sensor.getHeading()) >= targetAngle - 5)) {
                stopRobot(robot);
                telemetry.addData("REACHED ANGLE", list.get(robot.v_gyro_sensor.getHeading()));
                telemetry.update();
                goalReached = true;
            }
            if (goalReached) {
                robot.v_motor_left_drive.setPower(0);
                robot.v_motor_right_drive.setPower(0);
                robot.v_motor_left_drive_back.setPower(0);
                robot.v_motor_right_drive_back.setPower(0);
                break ;
            }
            if (!goalReached) {
                robot.v_motor_left_drive.setPower(turnPower);
                robot.v_motor_right_drive.setPower(-turnPower);
                robot.v_motor_left_drive_back.setPower(turnPower);
                robot.v_motor_right_drive_back.setPower(-turnPower);
            }
        }
        reset_drive_encoders(robot);
        return goalReached;
    }



    public boolean turnRightGyroByAngle(Shockwave71712017Hardware robot, int startAngle, int deltaAngle) {
        double turnPower = 0;
        boolean goalReached = false;

        if (deltaAngle + startAngle - robot.v_gyro_sensor.getHeading() > 180) {
            turnPower = 0.5;
        } else {
            turnPower = -0.5;
        }
        while (goalReached == false  && opModeIsActive() == true) {
            if (Math.abs(list.get(robot.v_gyro_sensor.getHeading()) + startAngle - deltaAngle) <= 0 ){
//            if ((robot.v_gyro_sensor.getHeading() <= Angle + 5) && (robot.v_gyro_sensor.getHeading() >= Angle - 5)) {
                goalReached = true;
            }
            if (goalReached) {
                robot.v_motor_left_drive.setPower(0);
                robot.v_motor_right_drive.setPower(0);
                robot.v_motor_left_drive_back.setPower(0);
                robot.v_motor_right_drive_back.setPower(0);
                break ;
            }
            if (!goalReached) {
                robot.v_motor_left_drive.setPower(turnPower);
                robot.v_motor_right_drive.setPower(-turnPower);
                robot.v_motor_left_drive_back.setPower(turnPower);
                robot.v_motor_right_drive_back.setPower(-turnPower);
            }
        }
        return goalReached;


    }

    public boolean turnRightGyro(Shockwave71712017Hardware robot, double Angle, int startAngle) {
        //reset_drive_encoders(robot);
        int delta ;

        telemetry.addData("Start Angle", startAngle);
        telemetry.update();
        sleep(3000);
        double turnPower = 0;
        boolean goalReached = false;
        if (Angle - robot.v_gyro_sensor.getHeading() > 180) {
            turnPower = 0.3;
        } else {
            turnPower = -0.3;
        }
        while(!goalReached && opModeIsActive()) {
            delta = robot.v_gyro_sensor.getHeading() - startAngle ;
            telemetry.addData("Difference:", delta);
            telemetry.update();
            sleep(200);
            if ((delta <= Angle + 3) && (delta >= Angle - 3))
                goalReached = true;
            robot.v_motor_left_drive.setPower(turnPower);
            robot.v_motor_right_drive.setPower(-turnPower);
            robot.v_motor_left_drive_back.setPower(turnPower);
            robot.v_motor_right_drive_back.setPower(-turnPower);

        }
        robot.v_motor_left_drive.setPower(0);
        robot.v_motor_right_drive.setPower(0);
        robot.v_motor_left_drive_back.setPower(0);
        robot.v_motor_right_drive_back.setPower(0);


        return goalReached;


    }


    public boolean turnLeftGyro(Shockwave71712017Hardware robot, double Angle, int startAngle) {
        //reset_drive_encoders(robot);

        telemetry.addData("Start Angle", startAngle);
        telemetry.update();
        sleep(3000);
        double turnPower = 0;
        boolean goalReached = false;
        if (Angle - robot.v_gyro_sensor.getHeading() > 180) {
            turnPower = -0.5;
        } else {
            turnPower = 0.5;
        }
        while(!goalReached && opModeIsActive()) {
            telemetry.addData("Difference:", robot.v_gyro_sensor.getHeading() - startAngle);
            telemetry.update();
           // sleep(200);
            if ((robot.v_gyro_sensor.getHeading() - startAngle <= Angle + 3) && (robot.v_gyro_sensor.getHeading() - startAngle>= Angle - 3))
                goalReached = true;
            robot.v_motor_left_drive.setPower(turnPower);
            robot.v_motor_right_drive.setPower(-turnPower);
            robot.v_motor_left_drive_back.setPower(turnPower);
            robot.v_motor_right_drive_back.setPower(-turnPower);

        }
        robot.v_motor_left_drive.setPower(0);
        robot.v_motor_right_drive.setPower(0);
        robot.v_motor_left_drive_back.setPower(0);
        robot.v_motor_right_drive_back.setPower(0);


        return goalReached;
    }

    public void resetAngle(double last){
        lastAngle = last;
        globalAngle = 0;
    }

    public double pControlTurn(Shockwave71712017Hardware robot, double degrees, double power, PIDController pidRotate){
        // restart imu angle tracking.
        resetAngle(robot.v_gyro_sensor.getHeading());

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.v_gyro_sensor.getHeading() == 0)
            {
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(robot.v_gyro_sensor.getHeading()); // power will be - on right turn.
                turnLeft(robot, 10, power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(robot.v_gyro_sensor.getHeading()); // power will be + on left turn.
                turnRight(robot, 10, power);

            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        stopRobot(robot);

        //rotation =robot.v_gyro_sensor.getHeading();

        // wait for rotation to stop.
        sleep(300);

        // reset angle tracking on new heading.
        resetAngle(robot.v_gyro_sensor.getHeading());
        return 0;
    }


    public boolean moveTillColor(Shockwave71712017Hardware robot, int color, double power)
    {

        robot.colorSensor.enableLed(true);
        while(opModeIsActive()) {
            if(color == RED)
            {
                if(robot.colorSensor.red() > 4)
                    break;
            }
            else
                if(robot.colorSensor.blue() > 4)
                    break;
            telemetry.addData("Red Color", robot.colorSensor.red());
            telemetry.addData("blue Color", robot.colorSensor.blue());
            telemetry.update();
            goForward(robot, power);
        }
        // sleep(2000);
        stopRobot(robot);
        robot.colorSensor.enableLed(false);
        return true;
    }
    public boolean moveTillColorMec(Shockwave71712017Hardware robot, int color, double power)
    {

        robot.colorSensor.enableLed(true);
        while(opModeIsActive()) {
            if(color == RED)
            {
                if(robot.colorSensor.red() > 4)
                    break;

            }
            else
            if(robot.colorSensor.blue() > 4)
                break;
            telemetry.addData("Red Color", robot.colorSensor.red());
            telemetry.update();
//            telemetry.addData("blue Color", robot.colorSensor.blue());
//            telemetry.update();
            moveLeftUsingMechanumWithoutN(robot, power);
        }
        // sleep(2000);
        stopRobot(robot);
        robot.colorSensor.enableLed(false);
        return true;
    }
    public double getDistanceFromWallFront(Shockwave71712017Hardware robot){
        return robot.v_range_sensor.getDistance(DistanceUnit.CM);
    }
    public void moveToDistanceToWall(Shockwave71712017Hardware robot, double distance){
        while(getDistanceFromWallFront(robot)>=distance)
            goForward(robot, .2);
    }
    public boolean moveTillDistance(Shockwave71712017Hardware robot, double distance, double power) {  // distance in cm
        double rangeDistance;
//get distance from the sensor
        //keep moving until reach x distance (while)
//        while (opModeIsActive()) {
//            telemetry.addData("cm", "%.2f cm", robot.v_range_sensor.getDistance(DistanceUnit.CM));
//
//        }
        int multiplier = 1;

        rangeDistance = robot.v_range_sensor.getDistance(DistanceUnit.CM);
        if (rangeDistance < distance)
            return true ;
        while (rangeDistance > distance && opModeIsActive()){
            telemetry.addData("cm", "%.2f cm", rangeDistance);
            telemetry.update() ;
            goForward(robot, power);
            rangeDistance = robot.v_range_sensor.getDistance(DistanceUnit.CM);
        }
        stopRobot(robot);
        return true;
    }
    public boolean moveTillDistanceAway(Shockwave71712017Hardware robot, double distance, double power){
        double rangeDistance;
        int multiplier = 1;

        rangeDistance = robot.v_range_sensor.getDistance(DistanceUnit.CM);
        if (rangeDistance > distance)
            return true ;
        while (rangeDistance < distance && opModeIsActive()){
            telemetry.addData("cm", "%.2f cm", rangeDistance);
            telemetry.update() ;
            goForward(robot, power);
            rangeDistance = robot.v_range_sensor.getDistance(DistanceUnit.CM);
        }
        stopRobot(robot);
        return true;
    }

    public boolean moveTillDistanceScan(Shockwave71712017Hardware robot, double distance, double armDistance, String side, double power) {  // distance in cm
         double rangeDistance;
//get distance from the sensor
        //keep moving until reach x distance (while)
//        while (opModeIsActive()) {
//            telemetry.addData("cm", "%.2f cm", robot.v_range_sensor.getDistance(DistanceUnit.CM));
//
//        }
        int multiplier = 1;
        if(side.equals("red"))
            multiplier = -1;
        rangeDistance = robot.v_range_sensor.getDistance(DistanceUnit.CM);
        distance = distance+multiplier*armDistance;
        if (rangeDistance < distance)
            return true ;
        while (rangeDistance > distance && opModeIsActive()){
            telemetry.addData("cm", "%.2f cm", rangeDistance);
            telemetry.update() ;
            goForward(robot, -multiplier*power);
            rangeDistance = robot.v_range_sensor.getDistance(DistanceUnit.CM);
        }
        stopRobot(robot);
        return true;
    }
    public void moveAwayTillDistance(Shockwave71712017Hardware robot, double distance, double power){
        double rangeDistance;
        rangeDistance = robot.rightSideDistanceSensor.getDistance(DistanceUnit.CM);
        if (rangeDistance < distance)
            return ;
        while(rangeDistance>distance&& opModeIsActive()){
            telemetry.addData("cm", "%.2f cm", rangeDistance);
            telemetry.update() ;
            moveLeftUsingMechanumWithoutN(robot, power);
            rangeDistance = robot.rightSideDistanceSensor.getDistance(DistanceUnit.CM);

        }
        stopRobot(robot);
        return;
    }

    public boolean detectWhiteLine(Shockwave71712017Hardware robot) { //Worked on 2/4/17
        lightValue = robot.v_optical_sensor.getLightDetected();
        telemetry.addData("lightValue", lightValue);
        telemetry.update();
        //idle();
        startAngle = list.get(robot.v_gyro_sensor.getHeading());
        robot.v_motor_left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (lightValue < .08 && opModeIsActive()) {   //.014
     //       while (opModeIsActive()) {
                telemetry.addData("lightValue", lightValue);
                telemetry.update();
                // goForwardUsingGyro(robot, 1000, -.5, startAngle);
            robot.v_motor_left_drive.setPower(-0.1);
            robot.v_motor_right_drive.setPower(-0.1);
            robot.v_motor_left_drive_back.setPower(-0.1*0.66);
            robot.v_motor_right_drive_back.setPower(-0.1*0.66);
            lightValue = robot.v_optical_sensor.getLightDetected();
            }
        robot.v_motor_left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopRobot(robot);
        return true;
    }

    public boolean moveRightUsingMecanumWheels(Shockwave71712017Hardware robot, double numOfRotations, double power) {
        run_using_encoders(robot);
        robot.v_motor_right_drive.setPower(-power);
        robot.v_motor_right_drive_back.setPower(power);
        robot.v_motor_left_drive.setPower(power);
        robot.v_motor_left_drive_back.setPower(-power);
        reset_drive_encoders(robot);
        telemetry.addData("Moving Right", "Return true");
        return true;
    }

    public boolean moveLeftUsingMecanumWheels(Shockwave71712017Hardware robot, double numOfRotations, double power) {
        reset_drive_encoders(robot);
        run_using_encoders(robot);
        while (isEncoderValueReached(numOfRotations, robot.v_motor_right_drive) == false && opModeIsActive()) {
            robot.v_motor_right_drive.setPower(power);
            robot.v_motor_right_drive_back.setPower(-power);
            robot.v_motor_left_drive.setPower(-power);
            robot.v_motor_left_drive_back.setPower(power);
        }
        robot.v_motor_left_drive.setPower(0);
        robot.v_motor_right_drive.setPower(0);
        robot.v_motor_left_drive_back.setPower(0);
        robot.v_motor_right_drive_back.setPower(0);
        reset_drive_encoders(robot);
        telemetry.addData("Moving Left", "Return true");
        return true;

    }
    public boolean moveLeftUsingMechanumWithoutN(Shockwave71712017Hardware robot, double power){
        reset_drive_encoders(robot);
        run_using_encoders(robot);
        robot.v_motor_right_drive.setPower(power);
        robot.v_motor_right_drive_back.setPower(-power);
        robot.v_motor_left_drive.setPower(-power);
        robot.v_motor_left_drive_back.setPower(power);
        return true;
    }
    public boolean foundationDown (Shockwave71712017Hardware robot){
        robot.v_servo_foundation_left.setPosition(1);
        robot.v_servo_foundation_right.setPosition(0);
        return true;
    }

    public boolean foundationUp (Shockwave71712017Hardware robot){
        robot.v_servo_foundation_left.setPosition(0.2);
        robot.v_servo_foundation_right.setPosition(0.8);
        return true;
    }

    public boolean goForward (Shockwave71712017Hardware robot,  double power) {
        reset_drive_encoders(robot);
        run_without_encoders(robot);
        robot.v_motor_right_drive.setPower(power);
        robot.v_motor_left_drive.setPower(power);
        robot.v_motor_right_drive_back.setPower(power);
        robot.v_motor_left_drive_back.setPower(power);
        return true;
    }

    //    public boolean openLift(Shockwave71712017Hardware robot)
//    {
//        robot.v_servo_push_one.setPosition(1.0);
//        robot.v_servo_push_two.setPosition(0.1);
//        robot.v_servo_push_top_left.setPosition(0.9);
//        robot.v_servo_push_top_right.setPosition(0.1);
//        return true;
//    }
//    public boolean closeLift(Shockwave71712017Hardware robot)
//    {
//        robot.v_servo_push_one.setPosition(0.3);
//        robot.v_servo_push_two.setPosition(0.6);
//        robot.v_servo_push_top_left.setPosition(0.9);
//        robot.v_servo_push_top_right.setPosition(0.1);
//        return true;
//    }
//    public void landRobot (Shockwave71712017Hardware robot)
//    {
//        robot.v_motor_latch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.v_motor_latch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        while ((isEncoderValueReached(18300, robot.v_motor_latch) == false) && (opModeIsActive() == true)) {
//            robot.v_motor_latch.setPower(1.0);
//        }
//        robot.v_motor_latch.setPower(0);
//
//        robot.v_motor_latch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }

//    public void dropClaim (Shockwave71712017Hardware robot)
//    {
//        robot.v_servo_claim.setPosition (0.7);
//        sleep (500 );
//        robot.v_servo_claim.setPosition (0.3);
//    }

    public void shockDelay()
    {
       // sleep (10000);
    }

  void scanInit () {
      // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
      JXPcameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

      JXPparameters = new VuforiaLocalizer.Parameters(JXPcameraMonitorViewId);
      JXPparameters.vuforiaLicenseKey = JXP_VUFORIA_KEY;
      JXPparameters.cameraDirection = CAMERA_CHOICE;
      if (CAMERA_CHOICE == BACK) {
          phoneYRotate = -90;
      } else {
          phoneYRotate = 90;
      }

      // Rotate the phone vertical about the X axis if it's in portrait mode
      if (PHONE_IS_PORTRAIT) {
          phoneXRotate = 90;
      }
      JXProbotFromCamera = OpenGLMatrix
              .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
              .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

      JXPvuforia = ClassFactory.getInstance().createVuforia(JXPparameters);

      telemetry.addData("after getInstance","after getInstance" );
      telemetry.update() ;

      // Load the data sets for the trackable objects. These particular data
      // sets are stored in the 'assets' part of our application.
      JXPtargetsSkyStone = JXPvuforia.loadTrackablesFromAsset("Skystone");
      telemetry.addData("Problem method call","load trackable" );
      telemetry.update() ;

  }
//
//    public void scan(Shockwave71712017Hardware robot, String side){
//
//        vScanner(robot, side);
//
//
//
//    } // end if(blueValue != 0)
//
//
    public int testInitScan(Shockwave71712017Hardware robot, String side, String iteration) {
        VuforiaTrackable stoneTarget = JXPtargetsSkyStone.get(0);
        stoneTarget.setName("Stone  Target");
        position = 1;
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(JXPtargetsSkyStone);
        telemetry.addData("Problem method call", "addAll");
        telemetry.update();

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(JXProbotFromCamera, JXPparameters.cameraDirection);
        }
        telemetry.addData("Problem method call", "setPhone Information");
        telemetry.update();


        JXPtargetsSkyStone.activate();
        while (!isStopRequested() && opModeIsActive()) {

            // check all the trackable targets to see which one (if any) is visible.
            JXPtargetVisible = false;
            boolean forward = true;
            boolean check1 = false;
            boolean check2 = false;
            boolean check3 = false;
            int movements = 1;
            int count = 0;
            double power;
            double powerForAligningPuller;
            sleep(10);

            if (side.equals("red")) {
                power = 0.15;
                powerForAligningPuller = 0.3;
            } else {
                power = -.2;
                powerForAligningPuller = 0.2;
            }
            count++;
            if(count > 1000)
            {
                break;
            }
//            while (!JXPtargetVisible && opModeIsActive()) {
//                if (robot.v_range_sensor.getDistance(DistanceUnit.INCH) < 6)
//                    break;
//                if (forward) {
////                    goForwardNRotations(robot, 250, power);
////                    sleep(250);
////                    telemetry.addData ("In forward", movements);
////                    telemetry.update();
//                    goForward(robot, power);
//                    //sleep (350);
//                } else {
////                    goForwardNRotations(robot, 250, -power);
////                    sleep(250);
//                    //              goForward(robot, -power);
//                    //sleep (350);
//                    //sleep(200);
//                }
//                movements++;

                for (VuforiaTrackable trackable : allTrackables) {
                    if (opModeIsActive() == false)
                        break;
                    while (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() && opModeIsActive()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        if (JXPtargetVisible == false)
                            stopRobot(robot);
                        JXPtargetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            JXPlastLocation = robotLocationTransform;
                        }

                        VectorF translation = JXPlastLocation.getTranslation();
                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(JXPlastLocation, EXTRINSIC, XYZ, DEGREES);
                        float xcoord = translation.get(0) / mmPerInch; //location once it detects skystone
                        float ycoord = translation.get(1) / mmPerInch; //location once it detects skystone
                        float zcoord = translation.get(2) / mmPerInch;
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                        telemetry.addData("xcoord", xcoord);
                        telemetry.addData("ycoord", ycoord);
                        telemetry.update();
                        if(ycoord > 2)
                        {
                            position = 1;
                            if(side.equals("red"))
                                position = 3;
                            telemetry.addData("position is 1", "");
                            telemetry.update();

                            //if(iteration.equals("second"))
                                //return position;
                            goForwardNRotations(robot, 50, -.2);
                            //return position;
                        }
                        else if(ycoord < -2)
                        {
                            position = 3;
                            if(side.equals("red"))
                                position = 1;
                            telemetry.addData("position is 3", "");
                            telemetry.update();
                            //return position;
                            goForwardNRotations(robot, 50, .2);

                            //return position;
                        }
                        else
                        {
                            position = 2;
                            telemetry.addData("position is 2", "");
                            telemetry.update();
                            return position;
                            //return (int)ycoord;
                        }

                    }

                }
            }


            return position;
        }


    public void JXPScanner(Shockwave71712017Hardware robot, String side){
        //  Instantiate the Vuforia engine
//        JXPvuforia = ClassFactory.getInstance().createVuforia(JXPparameters);
//
//        telemetry.addData("after getInstance","after getInstance" );
//        telemetry.update() ;
//
//        // Load the data sets for the trackable objects. These particular data
//        // sets are stored in the 'assets' part of our application.
//        JXPtargetsSkyStone = JXPvuforia.loadTrackablesFromAsset("Skystone");
//        telemetry.addData("Problem method call","load trackable" );
//        telemetry.update() ;

        VuforiaTrackable stoneTarget = JXPtargetsSkyStone.get(0);
        stoneTarget.setName("Stone  Target");
//        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
//        blueRearBridge.setName("Blue Rear Bridge");
//        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
//        redRearBridge.setName("Red Rear Bridge");
//        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
//        redFrontBridge.setName("Red Front Bridge");
//        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
//        blueFrontBridge.setName("Blue Front Bridge");
//        VuforiaTrackable red1 = targetsSkyStone.get(5);
//        red1.setName("Red Perimeter 1");
//        VuforiaTrackable red2 = targetsSkyStone.get(6);
//        red2.setName("Red Perimeter 2");
//        VuforiaTrackable front1 = targetsSkyStone.get(7);
//        front1.setName("Front Perimeter 1");
//        VuforiaTrackable front2 = targetsSkyStone.get(8);
//        front2.setName("Front Perimeter 2");
//        VuforiaTrackable blue1 = targetsSkyStone.get(9);
//        blue1.setName("Blue Perimeter 1");
//        VuforiaTrackable blue2 = targetsSkyStone.get(10);
//        blue2.setName("Blue Perimeter 2");
//        VuforiaTrackable rear1 = targetsSkyStone.get(11);
//        rear1.setName("Rear Perimeter 1");
//        VuforiaTrackable rear2 = targetsSkyStone.get(12);
//        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(JXPtargetsSkyStone);
        telemetry.addData("Problem method call","addAll" );
        telemetry.update() ;

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

//        //Set the position of the bridge support targets with relation to origin (center of field)
//        blueFrontBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));
//
//        blueRearBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));
//
//        redFrontBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, -bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));
//
//        redRearBridge.setLocation(OpenGLMatrix
//                .translation(bridgeX, -bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));
//
//        //Set the position of the perimeter targets with relation to origin (center of field)
//        red1.setLocation(OpenGLMatrix
//                .translation(quadField, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        red2.setLocation(OpenGLMatrix
//                .translation(-quadField, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        front1.setLocation(OpenGLMatrix
//                .translation(-halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
//
//        front2.setLocation(OpenGLMatrix
//                .translation(-halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
//
//        blue1.setLocation(OpenGLMatrix
//                .translation(-quadField, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//
//        blue2.setLocation(OpenGLMatrix
//                .translation(quadField, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//
//        rear1.setLocation(OpenGLMatrix
//                .translation(halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
//
//        rear2.setLocation(OpenGLMatrix
//                .translation(halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.


        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.

        //stonetarget.getListener()

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(JXProbotFromCamera, JXPparameters.cameraDirection);
        }
        telemetry.addData("Problem method call","setPhone Information" );
        telemetry.update() ;

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        JXPtargetsSkyStone.activate();
        while (!isStopRequested() && opModeIsActive()) {

            // check all the trackable targets to see which one (if any) is visible.
            JXPtargetVisible = false;
            boolean forward = true;
            int movements = 1;
            double power;
            double  powerForAligningPuller ;


            if(side.equals("red")) {
                power = 0.15;
                powerForAligningPuller = 0.3;
            }else {
                power = -.2;
                powerForAligningPuller = 0.2;
            }
            while(!JXPtargetVisible&& opModeIsActive()) {
               if (robot.v_range_sensor.getDistance(DistanceUnit.INCH) < 6 )
                    break ;
                if (forward)
                {
//                    goForwardNRotations(robot, 250, power);
//                    sleep(250);
//                    telemetry.addData ("In forward", movements);
//                    telemetry.update();
                    goForward(robot, power);
                    //sleep (350);
                }
                else
                {
//                    goForwardNRotations(robot, 250, -power);
//                    sleep(250);
      //              goForward(robot, -power);
                    //sleep (350);
                    //sleep(200);
                }
                movements++;

                for (VuforiaTrackable trackable : allTrackables) {
                    if (opModeIsActive()==false)
                        break ;
                    while (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() && opModeIsActive()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        if (JXPtargetVisible == false )
                            stopRobot(robot) ;
                        JXPtargetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            JXPlastLocation = robotLocationTransform;
                        }

                        VectorF translation = JXPlastLocation.getTranslation();
                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(JXPlastLocation, EXTRINSIC, XYZ, DEGREES);
                        float xcoord = translation.get(0) / mmPerInch; //location once it detects skystone
                        float ycoord = translation.get(1) / mmPerInch; //location once it detects skystone
                        float zcoord = translation.get(2) / mmPerInch;
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                        telemetry.addData ("xcoord", xcoord);
                        telemetry.addData ("ycoord", ycoord);
                        telemetry.update();
//                        sleep (2000);

                        // constantly scanning for the position instead of using the xcoord just one time
                        if (!JXPxcoordDone) {
                            if (xcoord < -5.5) {//-8 moved not close enough
                                moveLeftUsingMechanumWithoutN(robot, -.7);

                            } else
                            {

                                stopRobot(robot);

                                if ( (robot.v_gyro_sensor.getHeading() - startAngle) > 180)
                                    turnGyroSpecialPivotToATargetAngle(robot,startAngle,TURN_LEFT,1,0);
                                else if ( (robot.v_gyro_sensor.getHeading() - startAngle) < 180)
                                    turnGyroSpecialPivotToATargetAngle(robot,startAngle,TURN_RIGHT,1,0);

                                JXPxcoordDone = true;
                            }
                        }
                        else {//-8 was not moving far enough to pick up skystone, became out of range
                            //if ((ycoord < -4) && !JXPycoordDone ) { //10, 20 didn't move enough (try moving 40 or 50)
                            goForwardNRotations(robot, 425, powerForAligningPuller );
                                telemetry.addData("moving y: ", ycoord);
                                telemetry.update();
                                sleep(500);


                        }


                        //if (JXPycoordDone && JXPxcoordDone)
//                        {
//                            if (side.equals("Red"))
//                            goForwardNRotations(robot, 300, powerForAligningPuller );
//                            else
//                                goForwardNRotations(robot, 580, powerForAligningPuller );
//
//                             moveLeftUsingMecanumWheels(robot,300,-.3);
////                            while (robot.colorDistanceSensor.getDistance(DistanceUnit.CM )> 3)
////                                moveLeftUsingMechanumWithoutN(robot,-.3);
////                            stopRobot(robot);
//                            break;
//                        }
                    }
                }
            }

            // Provide feedback as to where the robot is located (if we know).
//            if (JXPtargetVisible) {
//                // express position (translation) of robot in inches.
//                VectorF translation = JXPlastLocation.getTranslation();
////                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
////                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//                // express the rotation of the robot in degrees.
//                Orientation rotation = Orientation.getOrientation(JXPlastLocation, EXTRINSIC, XYZ, DEGREES);
////                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//                float xcoord = translation.get(0) ; //location once it detects skystone
//                float zcoord = translation.get(2) ;
//                telemetry.addData("SLEEP x coord",xcoord);
//                telemetry.addData("SLEEP z coord",zcoord);
//                telemetry.update();
//                sleep (5000);
//
//
//                goForwardNRotations(robot,50, -.3);
////                for(int i = 0; i < 325; i++) //
////                    goForward(robot, -.05);
//                //for(int i = 0; i < 20; i++)
//                while(robot.colorDistanceSensor.getDistance(DistanceUnit.CM) > 21 )
//                {
//                    moveLeftUsingMecanumWheels(robot, 20, -.8);
//                }
//
//
//                //goForwardNRotations(robot, xcoord+325, -.3);
//                //moveLeftUsingMecanumWheels(robot, zcoord -20, -1);
//            }
//            else {
//                telemetry.addData("Visible Target", "none");
//            }

            break ;
        } // while

        // Disable Tracking when we are done;
        JXPtargetsSkyStone.deactivate();
    }
}
