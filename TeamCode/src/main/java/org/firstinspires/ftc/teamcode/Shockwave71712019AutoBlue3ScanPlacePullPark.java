package org.firstinspires.ftc.teamcode;

/* DOGCV
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
*/
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

// @Disabled

//CREATED 11/4/18
@Autonomous(name="ABlue3ScanPlacePark", group ="Blue")

public class Shockwave71712019AutoBlue3ScanPlacePullPark extends Shockwave71712017CommonLinearOp{

    /* Declare OpMode members. */
    Shockwave71712017Hardware robot = new Shockwave71712017Hardware();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    static final double WHITE_THRESHOLD = 0.5;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.5;
    float hsvValues[] = {0F, 0F, 0F};
    int gameStartAngle;
    int startAnlge;
    int currentAngle;
    double servoPosition;
    //Elapsed time and measurement constants
    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor


// DOGCV    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    // **********SCANNING***********
    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;  //Back if using webcam
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private static final String VUFORIA_KEY =
            "AVcokEn/////AAAAGYxCoQ/keUsCglbU/LG4i8c/L2+TX9r70WyGPbO2JSTjA6hUGU5WZiYVyZRBFXi09YrcYeairjLBq1DGwFV9ZgDZWNb5+e5xm7oRimAQBladvj6vwzHbaP3TKemPgFd9ANPhxptpUKuOE/y4P2SXKnQUNiJZjFVvC2NV3OxSc3FzdQ45nsqJkgH0/dY1nuBbYM50uMy4fzsUEUHmSLyrLsQS8vXOdFDdcBcc4TBEiqd2aHbGHafpM1wz2KRzmN3XgDDBC2u14fkNsM4vmP1qwFPvda+3LaC6U/i+6obHQ2tG/CKXjq84IZnudFYLbmClhuacZaGbi8yNkiZJwwzPm7Bv0Ocd9B2560qJNea8B9gw";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //Detector object
//DOGCV    private GoldAlignDetector detector; // = new GoldAlignDetector();


    List<Integer> list = Arrays.asList(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, -179, -178, -177, -176, -175, -174, -173, -172, -171, -170, -169, -168, -167, -166, -165, -164, -163, -162, -161, -160, -159, -158, -157, -156, -155, -154, -153, -152, -151, -150, -149, -148, -147, -146, -145, -144, -143, -142, -141, -140, -139, -138, -137, -136, -135, -134, -133, -132, -131, -130, -129, -128, -127, -126, -125, -124, -123, -122, -121, -120, -119, -118, -117, -116, -115, -114, -113, -112, -111, -110, -109, -108, -107, -106, -105, -104, -103, -102, -101, -100, -99, -98, -97, -96, -95, -94, -93, -92, -91, -90, -89, -88, -87, -86, -85, -84, -83, -82, -81, -80, -79, -78, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60, -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1);

    /*Autonomous stuff
     * move robot down (latch up)
     * move out of the hook
     * move forward out of the region
     * use vuforia, locate where the robot is
     * angle robot towards the image, use whatever location stuff and arrange itself
     * turn around and do the gold detection thing
     * move out of the range for gold and silver minerals, go towards team zone
     * put down marker
     * go towards crater
     * "park"
     * do teleop
     */

/*
        IMPORTANT VALUES:
        - Moving Forward 1 block using GoForwardNRotations: 1000

 */

    final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
    VuforiaLocalizer.Parameters parameters;
    OpenGLMatrix robotFromCamera;
    String side = "blue";

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        double sidemultiplier;
        if(side.equals("red"))
            sidemultiplier = 1;
        else
            sidemultiplier = -1;
        //MARIAN
        //parameters.cameraName = webcamName;
//        robot.v_servo_skystone_grabber.setPosition(1);
//        robot.v_servo_skystone_puller.setPosition(0);

//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection   = CAMERA_CHOICE;
//        if (CAMERA_CHOICE == BACK) {
//            phoneYRotate = -90;
//        } else {
//            phoneYRotate = 90;
//        }
//
//        // Rotate the phone vertical about the X axis if it's in portrait mode
//        if (PHONE_IS_PORTRAIT) {
//            phoneXRotate = 90 ;
//        }
//        robotFromCamera = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        scanInit();
        while (robot.v_gyro_sensor.isCalibrating()&& opModeIsActive())
            robot.v_gyro_sensor.calibrate();
        startAngle = robot.v_gyro_sensor.getHeading();

        waitForStart();
        //skystone
       // JXP robot.v_servo_skystone_puller.setPosition(0);
        //moves toward the block

        moveLeftUsingMecanumWheels(robot, 2050, -0.5);

        /*
         * Make robot straight
         */
        telemetry.addData("headeding",robot.v_gyro_sensor.getHeading() );
        telemetry.addData("start angle",startAngle);
        telemetry.update();
        if ( (robot.v_gyro_sensor.getHeading() - startAngle) > 180)
            turnGyroSpecialPivotToATargetAngle(robot,startAngle,TURN_LEFT,1,0);
        else if ( (robot.v_gyro_sensor.getHeading() - startAngle) < 180)
            turnGyroSpecialPivotToATargetAngle(robot,startAngle,TURN_RIGHT,1,0);
        /* END ROBOT Stratight*/


        //scans
        JXPScanner(robot, side);

        robot.v_servo_skystone_puller.setPosition(0.85);
        telemetry.addData("puller down", 0);
        telemetry.update();

        sleep(500);
        //grabs the block
        robot.v_servo_skystone_grabber.setPosition(0);
        sleep(1000);

        robot.v_servo_skystone_puller.setPosition(.2); // Lift the puller a little

        telemetry.addData("grabber down", 0);
        telemetry.update();
        //moves the robot towards the wall
        moveLeftUsingMecanumWheels(robot,500,.5);
        telemetry.addData("move right", 0);
        telemetry.update();

        /*
         * Make robot straight
         */
        telemetry.addData("headeding",robot.v_gyro_sensor.getHeading() );
        telemetry.addData("start angle",startAngle);
        telemetry.update();
        if ( (robot.v_gyro_sensor.getHeading() - startAngle) > 180)
            turnGyroSpecialPivotToATargetAngle(robot,startAngle,TURN_LEFT,1,0);
        else if ( (robot.v_gyro_sensor.getHeading() - startAngle) < 180)
            turnGyroSpecialPivotToATargetAngle(robot,startAngle,TURN_RIGHT,1,0);
        /* END ROBOT Stratight*/

//        sleep(2000);
        //moves to the color
        moveTillColor(robot, BLUE, 0.2);
        telemetry.addData("between off color sensor and forward after line", 0);
        telemetry.update();
        //moves forward
        goForwardNRotations(robot, 2500, 0.9);
        moveLeftUsingMecanumWheels(robot, 600, -0.2);
        sleep(500);
        robot.v_servo_skystone_puller.setPosition(.8);
        //drops the skystone
        robot.v_servo_skystone_grabber.setPosition(0.8);
        telemetry.addData("grabber up", 0);
        telemetry.update();

        sleep(500);
        robot.v_servo_skystone_puller.setPosition(0);
        telemetry.addData("grabber down", 0);
        telemetry.update();

        //moves to the color
        //moveTillColor(robot, BLUE, -0.2); // moving back
        goForwardNRotations(robot, 2100, -.9);
    }
    public void restofProg()
    {
        telemetry.addData("next", "step");
        telemetry.update();
        sleep(5000);
    }

//    public void scan(){
//
//        vScanner(side);
//
//
//
//    } // end if(blueValue != 0)
//
//    public void vScanner(String side){
////MAzRian
//      //  webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); //haven't put webcam on hardware map yet
//
//
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
//         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
//         */
//
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Load the data sets for the trackable objects. These particular data
//        // sets are stored in the 'assets' part of our application.
//        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
//
//        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
//        stoneTarget.setName("Stone  Target");
////        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
////        blueRearBridge.setName("Blue Rear Bridge");
////        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
////        redRearBridge.setName("Red Rear Bridge");
////        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
////        redFrontBridge.setName("Red Front Bridge");
////        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
////        blueFrontBridge.setName("Blue Front Bridge");
////        VuforiaTrackable red1 = targetsSkyStone.get(5);
////        red1.setName("Red Perimeter 1");
////        VuforiaTrackable red2 = targetsSkyStone.get(6);
////        red2.setName("Red Perimeter 2");
////        VuforiaTrackable front1 = targetsSkyStone.get(7);
////        front1.setName("Front Perimeter 1");
////        VuforiaTrackable front2 = targetsSkyStone.get(8);
////        front2.setName("Front Perimeter 2");
////        VuforiaTrackable blue1 = targetsSkyStone.get(9);
////        blue1.setName("Blue Perimeter 1");
////        VuforiaTrackable blue2 = targetsSkyStone.get(10);
////        blue2.setName("Blue Perimeter 2");
////        VuforiaTrackable rear1 = targetsSkyStone.get(11);
////        rear1.setName("Rear Perimeter 1");
////        VuforiaTrackable rear2 = targetsSkyStone.get(12);
////        rear2.setName("Rear Perimeter 2");
//
//        // For convenience, gather together all the trackable objects in one easily-iterable collection */
//        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//        allTrackables.addAll(targetsSkyStone);
//
//        /**
//         * In order for localization to work, we need to tell the system where each target is on the field, and
//         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
//         * Transformation matrices are a central, important concept in the math here involved in localization.
//         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
//         * for detailed information. Commonly, you'll encounter transformation matrices as instances
//         * of the {@link OpenGLMatrix} class.
//         *
//         * If you are standing in the Red Alliance Station looking towards the center of the field,
//         *     - The X axis runs from your left to the right. (positive from the center to the right)
//         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
//         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
//         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
//         *
//         * Before being transformed, each target image is conceptually located at the origin of the field's
//         *  coordinate system (the center of the field), facing up.
//         */
//
//        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
//        // Rotated it to to face forward, and raised it to sit on the ground correctly.
//        // This can be used for generic target-centric approach algorithms
//        stoneTarget.setLocation(OpenGLMatrix
//                .translation(0, 0, stoneZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
////        //Set the position of the bridge support targets with relation to origin (center of field)
////        blueFrontBridge.setLocation(OpenGLMatrix
////                .translation(-bridgeX, bridgeY, bridgeZ)
////                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));
////
////        blueRearBridge.setLocation(OpenGLMatrix
////                .translation(-bridgeX, bridgeY, bridgeZ)
////                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));
////
////        redFrontBridge.setLocation(OpenGLMatrix
////                .translation(-bridgeX, -bridgeY, bridgeZ)
////                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));
////
////        redRearBridge.setLocation(OpenGLMatrix
////                .translation(bridgeX, -bridgeY, bridgeZ)
////                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));
////
////        //Set the position of the perimeter targets with relation to origin (center of field)
////        red1.setLocation(OpenGLMatrix
////                .translation(quadField, -halfField, mmTargetHeight)
////                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
////
////        red2.setLocation(OpenGLMatrix
////                .translation(-quadField, -halfField, mmTargetHeight)
////                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
////
////        front1.setLocation(OpenGLMatrix
////                .translation(-halfField, -quadField, mmTargetHeight)
////                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
////
////        front2.setLocation(OpenGLMatrix
////                .translation(-halfField, quadField, mmTargetHeight)
////                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
////
////        blue1.setLocation(OpenGLMatrix
////                .translation(-quadField, halfField, mmTargetHeight)
////                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
////
////        blue2.setLocation(OpenGLMatrix
////                .translation(quadField, halfField, mmTargetHeight)
////                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
////
////        rear1.setLocation(OpenGLMatrix
////                .translation(halfField, quadField, mmTargetHeight)
////                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
////
////        rear2.setLocation(OpenGLMatrix
////                .translation(halfField, -quadField, mmTargetHeight)
////                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//        //
//        // Create a transformation matrix describing where the phone is on the robot.
//        //
//        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
//        // Lock it into Portrait for these numbers to work.
//        //
//        // Info:  The coordinate frame for the robot looks the same as the field.
//        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
//        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
//        //
//        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
//        // pointing to the LEFT side of the Robot.
//        // The two examples below assume that the camera is facing forward out the front of the robot.
//
//        // We need to rotate the camera around it's long axis to bring the correct camera forward.
//
//
//        // Next, translate the camera lens to where it is on the robot.
//        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
//
//
//
//        /**  Let all the trackable listeners know where the phone is.  */
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
//        }
//
//        // WARNING:
//        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
//        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
//        // CONSEQUENTLY do not put any driving commands in this loop.
//        // To restore the normal opmode structure, just un-comment the following line:
//
//        // waitForStart();
//
//        // Note: To use the remote camera preview:
//        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
//        // Tap the preview window to receive a fresh image.
//
//        targetsSkyStone.activate();
//        while (!isStopRequested() && opModeIsActive()) {
//            // check all the trackable targets to see which one (if any) is visible.
//            targetVisible = false;
//            boolean forward = true;
//            int movements = 1;
//            double power;
//            if(side.equals("red"))
//                power = 0.3;
//            else
//                power = -0.3;
//            while(!targetVisible&& opModeIsActive()) {
//                if(movements%13 == 0)
//                    forward = false;
//                if (forward)
//                {
//                    goForwardNRotations(robot, 100, power);
//                    sleep(350);
//                }
//                else
//                {
//                    goForwardNRotations(robot,100, -power);
//                    sleep(350);
//                    //sleep(200);
//                }
//                movements++;
//
//                for (VuforiaTrackable trackable : allTrackables) {
//
//                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                        telemetry.addData("Visible Target", trackable.getName());
//                        targetVisible = true;
//
//                        // getUpdatedRobotLocation() will return null if no new information is available since
//                        // the last time that call was made, or if the trackable is not currently visible.
//                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//                        if (robotLocationTransform != null) {
//                            lastLocation = robotLocationTransform;
//                        }
//                        break;
//                    }
//                }
//            }
//
//            // Provide feedback as to where the robot is located (if we know).
//            if (targetVisible) {
//
//                // express position (translation) of robot in inches.
//                VectorF translation = lastLocation.getTranslation();
//                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//                // express the rotation of the robot in degrees.
//                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//                int count = 0;
//                float xcoord = translation.get(0) / mmPerInch; //location once it detects skystone
//                float zcoord = translation.get(2) / mmPerInch;
//                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//                while(xcoord<-13 || (xcoord>-8) && opModeIsActive()) {
////                    for (VuforiaTrackable trackable : allTrackables) {
////
////                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
////                            telemetry.addData("Visible Target", trackable.getName());
////                            targetVisible = true;
////
////                            // getUpdatedRobotLocation() will return null if no new information is available since
////                            // the last time that call was made, or if the trackable is not currently visible.
////                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
////                            if (robotLocationTransform != null) {
////                                lastLocation = robotLocationTransform;
////                            }
////                            break;
////                        }
////                    }
//                    if((xcoord > -8)) {
//                        goForwardNRotations(robot, 20,-.1);
//                        xcoord = xcoord - 1;
//                        telemetry.addData("In Problem Spot Needs to go Forward", "");
//                        telemetry.addData("x coord", xcoord);
//                        //telemetry.update();
//                    }
//                    if ((xcoord < -13)) {
//                        goForwardNRotations(robot, 20,.1);
//                        xcoord = xcoord + 1;
//                        telemetry.addData("In Problem Spot needs to go Backward", "");
//                        telemetry.addData("x coord",xcoord);
//                        //telemetry.update();
//                    }
//                    telemetry.update();
//                }
//                stopRobot(robot);
//                telemetry.addData("z coord",zcoord);
//                telemetry.update ();
//                sleep(1000);
//                while(zcoord > 2)
//                {
//                    telemetry.addData("z coord",zcoord);
//                    telemetry.update ();
//                    moveLeftUsingMecanumWheels(robot, 20, -1);
//                    zcoord = zcoord - 1;
//                }
////                while(lastLocation.getTranslation().get(3)/mmPerInch> 3){
////                    moveLeftUsingMecanumWheels(robot, 10, .2);
////                }
//            }
//            else {
//                telemetry.addData("Visible Target", "none");
//            }
//            telemetry.addData("Get X", getX());
//            telemetry.addData("Get Y", getY());
//            telemetry.addData("Get Z", getZ());
//            telemetry.update();
//            break;
//        }
//        // Disable Tracking when we are done;
//        targetsSkyStone.deactivate();
//    }
}

