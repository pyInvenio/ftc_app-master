package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;




public class Shockwave71712017Hardware
{
    public DcMotorController v_dc_motor_controller_drive;
    public double power_reduction_because_of_gears = 0.67 ;
    public DcMotor v_motor_left_drive;
    public DcMotor v_motor_right_drive;
    public DcMotor v_motor_latch;
    public DcMotor v_motor_left_drive_back;
    public DcMotor v_motor_right_drive_back;
    public DcMotor v_motor_intake_left;
    public DcMotor v_motor_intake_right;
    public DcMotor v_motor_pulley;
    public Servo v_servo_jewel;
    public Servo v_servo_rotator;
    public Servo v_servo_top_lift;
    public Servo v_servo_foundation_left;
    public Servo v_servo_foundation_right;
    public Servo v_servo_grabber;
    public Servo v_servo_skystone_puller;
    public Servo v_servo_skystone_grabber;
    public Servo v_servo_capstone;
    public CRServo v_servo_intakeHelp;
    public CRServo v_servo_intakeHelp2;

    public ModernRoboticsI2cGyro  v_gyro_sensor;
    public OpticalDistanceSensor v_optical_sensor;
    public ModernRoboticsI2cRangeSensor v_range_sensor;
    public ColorSensor colorSensor;
    public ColorSensor colorDistanceSensorColor;
    public DistanceSensor colorDistanceSensor;
    public DistanceSensor rightSideDistanceSensor;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    double intakeHelper2Stop = -0.05;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Shockwave71712017Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        try {
            //v_dc_motor_controller_drive = hwMap.dcMotorController.get("drive_front");
            //v_dc_motor_controller_drive_2 = hwMap.dcMotorController.get("drive_back");

       v_motor_left_drive = hwMap.dcMotor.get("left_front_drive");
            v_motor_right_drive = hwMap.dcMotor.get("right_front_drive");   // encoder
            v_motor_left_drive.setDirection(DcMotor.Direction.REVERSE); //changed from right_drive
            v_motor_right_drive_back = hwMap.dcMotor.get("right_back_drive");
            v_motor_left_drive_back = hwMap.dcMotor.get("left_back_drive");
            v_motor_left_drive_back.setDirection(DcMotor.Direction.REVERSE);
//            v_motor_aa = hwMap.dcMotor.get("motor_aa");

            v_motor_intake_right = hwMap.dcMotor.get("intake_right");
            v_motor_intake_left = hwMap.dcMotor.get("intake_left");
            v_motor_intake_left.setDirection(DcMotor.Direction.REVERSE);
            v_servo_intakeHelp = hwMap.crservo.get("servo_intake_help");
            v_servo_intakeHelp2 = hwMap.crservo.get("servo_intake_help2");
            v_servo_intakeHelp2.setPower(intakeHelper2Stop);

            v_motor_pulley = hwMap.dcMotor.get("pulley");
            v_servo_skystone_puller = hwMap.servo.get("skystone_puller"); //side BIG servo
            v_servo_skystone_puller.setPosition(.8);
            v_servo_skystone_grabber = hwMap.servo.get("skystone_grabber");//side
            v_servo_skystone_grabber.setPosition(0);
            v_servo_skystone_puller.setPosition(0.2);
            Thread.sleep(300);
            v_servo_grabber = hwMap.servo.get("grabber"); //lift
            v_servo_grabber.setPosition (0.3);
            v_servo_rotator = hwMap.servo.get("rotator");
            v_servo_rotator.setPosition(0.8);
            v_servo_top_lift = hwMap.servo.get("servo_top_lift");
            v_servo_top_lift.setPosition(0.9);
            v_servo_foundation_left = hwMap.servo.get("foundation_left");
            v_servo_foundation_left.setPosition(0.2);
            v_servo_foundation_right = hwMap.servo.get("foundation_right");
            v_servo_foundation_right.setPosition(0.8);

            v_servo_capstone = hwMap.servo.get("servo_capstone");
            v_servo_capstone.setPosition(0);
            colorSensor = hwMap.colorSensor.get("csensor");
            colorSensor.enableLed(false);
            colorDistanceSensor = hwMap.get(DistanceSensor.class, "color_distance");
            colorDistanceSensorColor = hwMap.get(ColorSensor.class, "color_distance");
            colorDistanceSensorColor.enableLed(false);
            rightSideDistanceSensor = hwMap.get(DistanceSensor.class, "right_side_distance");
            v_range_sensor= hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");

            //v_servo_top_lift.setPosition(0.3);
    /*
     *  SENSORS
     */
            v_gyro_sensor = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro_sensor");
//            v_range_sensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor");

        }
        catch (Exception e) {
               System.out.println("Catch");
        }

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

