/*
*/
package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

//import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name="TeleOp", group="7171")
//@Disabled
public class Shockwave71712017TeleOp extends OpMode {
//    double servo_one;
//    double servo_position_right;
//    double servo_position_left;
//    double servo_clamp;
//    double fExtendGearPower = 0.0 ;
    double ldScalingFactor = 1.0 ;
    boolean bGrabberPushed = true;
    boolean liftTurned = false;
    Shockwave71712017Hardware  robot   = new Shockwave71712017Hardware();

    double rotatorPosition = 1.0;

    double elapsedtimeg1 = 0;
    double elapsedtimeg2 = 0;
    long  lastTrigger4SkyStonePuller ;
    boolean counttimeg2 = false;
    boolean counttimeg1 = false;
    boolean topliftdown = false;

    double thresholdtime = 30;
    double intakeHelper2Stop = -0.05;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

   }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.init(hardwareMap);
        //        robot.v_servo_rotator.setPosition(rotatorPosition);
//        robot.v_servo_skystone_puller.setPosition(0.2);
//        robot.v_servo_foundation_puller.setPosition(.5);
//        robot.v_servo_top_lift.setPosition(.5);
//        robot.v_servo_skystone_grabber.setPosition(0);
    }
    long tickCountertape = System.currentTimeMillis();
    long prevtickCountertape = System.currentTimeMillis();
    boolean tickBooleantape = false;
    boolean tapeBool = false;
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        float l_gp1_left_stick_y = gamepad1.left_stick_y;
        float l_gp1_right_stick_y = gamepad1.right_stick_y;
        float gp1_right_trigger = gamepad1.right_trigger;
        float gp1_left_trigger = gamepad1.left_trigger;
        boolean gp1_right_bumper = gamepad1.right_bumper;
        boolean gp1_left_bumper = gamepad1.left_bumper;
        float l_gp1_right_stick_x = gamepad1.right_stick_x;
        double position;
        float gp2_left_trigger = gamepad2.left_trigger;
        float r_gp2_left_stick_y = gamepad2.left_stick_y;
        float r_gp2_right_stick_y = gamepad2.right_stick_y;
        float r_gp2_right_stick_x = gamepad2.right_stick_x;
        boolean gp2_left_bumper = gamepad2.left_bumper;
        boolean gp2_right_bumper = gamepad2.right_bumper;
        float gp2_right_trigger = gamepad2.right_trigger;

        boolean reducedspeed = false;
        boolean switchrightsticktotape = false;
        boolean gp2_y_button = gamepad2.y;
        boolean gp2_a_button = gamepad2.a;
        boolean gp2_b_button = gamepad2.b;
        boolean gp2_x_button = gamepad2.x;
        boolean gp1_y_button = gamepad1.y;
        boolean gp1_a_button = gamepad1.a;
        boolean gp1_x_button = gamepad1.x;
        boolean gp1_b_button = gamepad1.b;
        float stick_threshold = (float) 0.05;
        if (Math.abs(l_gp1_left_stick_y) <= stick_threshold) {
            l_gp1_left_stick_y = (float) 0.0;
        }
        if (Math.abs(l_gp1_right_stick_y) <= stick_threshold) {
            l_gp1_right_stick_y = (float) 0.0;
        }
        if (Math.abs(r_gp2_left_stick_y) <= stick_threshold) {
            r_gp2_left_stick_y = (float) 0.0;
        }
        if (Math.abs(r_gp2_right_stick_y) <= stick_threshold) {
            r_gp2_right_stick_y = (float) 0.0;
        }

        float yVal = gamepad1.left_stick_y;
        float xVal = gamepad1.left_stick_x;
        if (Math.abs(gamepad1.left_stick_x) <= stick_threshold) {
            xVal = (float) 0.0;
        }
        if (Math.abs(gamepad1.left_stick_x) <= stick_threshold) {
            xVal = (float) 0.0;
        }
        if (Math.abs(gamepad1.left_stick_y) <= stick_threshold) {
            yVal = (float) 0.0;
        }

//        double ldScalingFactor = 1.0 ;
        double left_stick_y = gamepad1.left_stick_y * ldScalingFactor;
        double left_stick_x = gamepad1.left_stick_x * ldScalingFactor;
        double right_stick_x = gamepad1.right_stick_x * ldScalingFactor;
        double right_stick_y = gamepad1.right_stick_y * ldScalingFactor;

        /*
         *******************************************************************************
         * GamePad #1 Controls
         *******************************************************************************
         */
        //      set_drive_power(l_gp1_left_stick_y, l_gp1_right_stick_y);
        set_drive_power_macanum(gamepad1);
        //  single_stick_drive(l_gp1_right_stick_y, l_gp1_right_stick_x);
        if (tapeBool){
            tickCountertape = SystemClock.currentThreadTimeMillis();

            if (tickBooleantape && tickCountertape - prevtickCountertape > 500 ) {
                robot.v_motor_measuring_tape.setPower(.3);
            } else if (!tickBooleantape && tickCountertape - prevtickCountertape >500) {
                robot.v_motor_measuring_tape.setPower(-.3);
            } else {
                robot.v_motor_measuring_tape.setPower(0);
                tickCountertape = 0;
                tapeBool = false;
            }
        }
        if (counttimeg1) {
            elapsedtimeg1 += 1;
            if (elapsedtimeg1 > thresholdtime) {
                elapsedtimeg1 = 0;
                counttimeg1 = false;
            }
        }
        if (counttimeg2) {
            elapsedtimeg2 += 1;
            if (elapsedtimeg2 > thresholdtime) {
                elapsedtimeg2 = 0;
                counttimeg2 = false;
            }

        }
//        if(gamepad1.x && !reducedspeed && elapsedtimeg1 == 0){
//            ldScalingFactor = 0.5;
//            reducedspeed = true;
//            counttimeg1 = true;
//        }
//        if(gamepad1.b && reducedspeed && elapsedtimeg1 == 0){
//            ldScalingFactor = 1.0;
//            reducedspeed = false;
//            counttimeg1 = true;
//        }

        if (gamepad1.x) {
            ldScalingFactor = 0.3;
        }
        if (gamepad1.b) {
            ldScalingFactor = 1.0;
        }
        if (gamepad1.y) {
            robot.v_servo_top_lift.setPosition(1);
            robot.v_servo_grabber.setPosition(0.4); // original 1.0
            // topliftdown = false;
        }
        if (gamepad1.a) {
            robot.v_servo_top_lift.setPosition(0.2);
            long start = System.currentTimeMillis();
            while (shockWait(400, start)) ;//orginal 0.9
            robot.v_servo_grabber.setPosition(0.1); // ORIGINAL : 0.2

            //topliftdown = true;
        }
        if (gamepad1.left_trigger > 0) {
            robot.v_servo_skystone_grabber.setPosition(0);
        }
        if (gamepad1.right_trigger > 0)
            robot.v_servo_skystone_grabber.setPosition(1);
        if (gamepad1.right_bumper)
            robot.v_servo_skystone_puller.setPosition(0.2); //uo
        if (gamepad1.left_bumper)
            robot.v_servo_skystone_puller.setPosition(0.8);  //down

        /*
         *******************************************************************************
         * GamePad #2 Controls
         *******************************************************************************
         */
        if (gamepad2.right_trigger > 0) {
            robot.v_servo_foundation_right.setPosition(0);
            robot.v_servo_foundation_left.setPosition(1.0);
        }
        if (gamepad2.left_trigger > 0) {
            robot.v_servo_foundation_right.setPosition(0.8);
            robot.v_servo_foundation_left.setPosition(0.2);
        }
        //TEMP
        robot.v_motor_intake_right.setPower(gamepad2.left_stick_y);
        robot.v_motor_intake_left.setPower(gamepad2.left_stick_y);
        //robot.v_motor_measuring_tape.setPower(gamepad2.left_stick_y);
        if (gamepad2.left_stick_y < 0) {
            robot.v_servo_intakeHelp.setPower(3.0); //CR Servo
            robot.v_servo_intakeHelp2.setPower(-1.0);
        } else if (gamepad2.left_stick_y > 0) {
            robot.v_servo_intakeHelp.setPower(-1.0); //CR Servo
            robot.v_servo_intakeHelp2.setPower(1.0);
        } else {
            robot.v_servo_intakeHelp.setPower(0.0);// CR Servo
            robot.v_servo_intakeHelp2.setPower(intakeHelper2Stop);
        }


        //lift arm turn 180
        /*if (gamepad2.b) {
            if (liftTurned && robot.v_servo_rotator.getPosition()>=1.0) {
                robot.v_servo_rotator.setPosition(0.0);
                liftTurned = false;
            }
            else if (!liftTurned&& robot.v_servo_rotator.getPosition()<1.0) {
                robot.v_servo_rotator.setPosition(1.0);
                liftTurned = true;
            }
         */
        if (gamepad2.right_bumper && elapsedtimeg2 == 0) {
            /*if(rotatorPosition<1.0)
                rotatorPosition+=.1;*/
            //robot.v_servo_rotator.setPosition(1.0);
            robot.v_servo_rotator.setPosition(1.0);
            counttimeg2 = true;
        }
        if (gamepad2.left_bumper && elapsedtimeg2 == 0) {
            if (rotatorPosition > 0.0)
                rotatorPosition -= .1;
            robot.v_servo_rotator.setPosition(0.0); //was 0.0 before - changed to align with robot
            counttimeg2 = true;
        }

        //lift grabber grab and release
        if (gamepad2.x)
            robot.v_servo_capstone.setPosition(1.0);
         else if (gamepad2.b)
            robot.v_servo_capstone.setPosition(0.10);

        if (gamepad2.a) {
            tickBooleantape = true;
            tickCountertape ++;
            tapeBool = true;
            prevtickCountertape = SystemClock.currentThreadTimeMillis();

        }
        if (gamepad2.y){
            tickBooleantape = false;
            tickCountertape ++;
            tapeBool = true;
            prevtickCountertape = SystemClock.currentThreadTimeMillis();
        }



        /*
        if(gamepad2.right_trigger > 0)  {
            robot.v_servo_grabber.setPosition(0.7);
        }
        if(gamepad2.left_trigger > 0)   {
            robot.v_servo_grabber.setPosition(0.2);
        }
        */
        //lift pulley
        double pulleypower = (double) gamepad2.right_stick_y;
        if (pulleypower > -.1 && pulleypower < .1) {
            pulleypower = 0;
        }
        if (!switchrightsticktotape) {
            if (pulleypower < 0)
                robot.v_motor_pulley.setPower(pulleypower * 0.5);
            else
                robot.v_motor_pulley.setPower(pulleypower * 0.3);


            // Loop
        }
        else{
            robot.v_motor_measuring_tape.setPower(pulleypower);
        }

    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


//    public boolean isEncoderValueReached( double numOfRotations, DcMotor motor)
//    {
//        telemetry.addData("Encoder Values : ", motor.getCurrentPosition());
//        telemetry.addData("numOfRotation : ", numOfRotations);
//
//        if (Math.abs (motor.getCurrentPosition ()) >= numOfRotations)
//        {
//            return true;
//        }
//        return false ;
//    }
//
//    public double getEncoderValue(  DcMotor motor)
//    {
//        return (Math.abs (motor.getCurrentPosition ()));
//    }

    void set_drive_power (double p_left_joystick_power, double p_right_joystick_power)
    {
        double ldReducedPowerLeft = (p_left_joystick_power *ldScalingFactor) ;// * (2.0/3.0);
        double ldReducedPowerRight = (p_right_joystick_power *ldScalingFactor);// * (2.0/3.0);
        /*v_motor_left_drive.setPower(p_left_joystick_power * ldScalingFactor);
        v_motor_right_drive.setPower(p_right_joystick_power * ldScalingFactor);
*/

    /*    v_motor_left_drive_back.setPower (ldReducedPowerLeft);
        v_motor_right_drive_back.setPower(ldReducedPowerRight);
*/
        if (ldReducedPowerLeft > -0.1 && ldReducedPowerLeft < 0.1 )
            ldReducedPowerLeft = 0.0 ;

        if (ldReducedPowerRight > -0.1 &&  ldReducedPowerRight < 0.1 )
            ldReducedPowerRight = 0.0 ;

        robot.v_motor_left_drive.setPower(ldReducedPowerLeft );      //Sets left front motor power
        robot.v_motor_right_drive.setPower(ldReducedPowerRight);      //Sets right front motor power
        robot.v_motor_left_drive_back.setPower(ldReducedPowerLeft );      //Sets left back motor power
        robot.v_motor_right_drive_back.setPower(ldReducedPowerRight);      //Sets right back motor power
    }

    void set_drive_power_macanum (Gamepad gamepad)
    {
        double left_stick_y = gamepad.left_stick_y * ldScalingFactor;
        double left_stick_x = gamepad.left_stick_x * ldScalingFactor;
        double right_stick_x = gamepad.right_stick_x * ldScalingFactor;
        double right_stick_y = gamepad.right_stick_y * ldScalingFactor;

        if (left_stick_y > -0.2 && left_stick_y < 0.2 )
            left_stick_y = 0.0 ;
        if (left_stick_x > -0.2 &&  left_stick_x < 0.2 )
            left_stick_x = 0.0 ;
        if (right_stick_x > -0.2 &&  right_stick_x < 0.2 )
            right_stick_x = 0.0 ;
        if (right_stick_y > -0.2 &&  right_stick_y < 0.2 )
            right_stick_y = 0.0 ;

        /*robot.v_motor_left_drive.setPower(left_stick_y  + left_stick_x+ right_stick_x );      //Sets left front motor power
        robot.v_motor_left_drive_back.setPower(left_stick_y - left_stick_x + right_stick_x );      //Sets left back motor power
        robot.v_motor_right_drive.setPower(left_stick_y - left_stick_x - right_stick_x );      //Sets right front motor power
        robot.v_motor_right_drive_back.setPower(left_stick_y + left_stick_x - right_stick_x ); */     //Sets right back motor power


//        robot.v_motor_left_drive_back.setPower(left_stick_y + left_stick_x - (right_stick_x*.66) );      //Sets left back motor power
//        robot.v_motor_right_drive.setPower(left_stick_y + left_stick_x + (right_stick_x*.66) );      //Sets right front motor power
//        robot.v_motor_left_drive.setPower(left_stick_y  - left_stick_x- (right_stick_x*.66) );      //Sets left front motor power
//        robot.v_motor_right_drive_back.setPower(left_stick_y - left_stick_x + (right_stick_x*.66) );      //Sets right back motor power

        robot.v_motor_left_drive_back.setPower(left_stick_y + left_stick_x - (right_stick_x) );      //Sets left back motor power
        robot.v_motor_right_drive.setPower(left_stick_y + left_stick_x + (right_stick_x) );      //Sets right front motor power
        robot.v_motor_left_drive.setPower(left_stick_y  - left_stick_x- (right_stick_x) );      //Sets left front motor power
        robot.v_motor_right_drive_back.setPower(left_stick_y - left_stick_x + (right_stick_x) );      //Sets right back motor power


    }

    public boolean shockWait(long duration, long start)
    {
         long currentTime = System.currentTimeMillis();
         if(currentTime < start + duration)
         {
             return true;
         }
         else
         {
             return false;
         }
    }

}
