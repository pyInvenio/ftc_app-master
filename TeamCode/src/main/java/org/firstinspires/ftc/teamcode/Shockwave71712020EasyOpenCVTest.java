package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;


@TeleOp
    public class Shockwave71712020EasyOpenCVTest extends LinearOpMode
    {
        /**
         * NB: we declare our camera as the {@link OpenCvInternalCamera} type,
         * as opposed to simply {@link OpenCvCamera}. This allows us to access
         * the advanced features supported only by the internal camera.
         */
        OpenCvInternalCamera phoneCam;

        @Override
        public void runOpMode()
        {
            telemetry.addData("recording", "");
            telemetry.update();
            sleep(500);
            /**
             * NOTE: Many comments have been omitted from this sample for the
             * sake of conciseness. If you're just starting out with EasyOpenCV,
             * you should take a look at {@link InternalCameraExample} or its
             * webcam counterpart, {@link WebcamExample} first.
             */

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
            phoneCam.openCameraDevice();
            phoneCam.setPipeline(new UselessColorBoxDrawingPipeline(new Scalar(255, 0, 0)));
            /*
             * We use the most verbose version of #startStreaming(), which allows us to specify whether we want to use double
             * (default) or single buffering. See the JavaDoc for this method for more details
             */
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT, OpenCvInternalCamera.BufferMethod.DOUBLE);

            /*
             * Demonstrate how to turn on the flashlight
             */
            //phoneCam.setFlashlightEnabled(true);

            /*
             * Demonstrate how to use the zoom. Here we zoom
             * in as much as is supported.
             */
            phoneCam.setZoom(phoneCam.getMaxSupportedZoom());

            /*
             * Demonstrate how to set the recording hint on the
             * camera hardware. See the JavDoc for this method
             * for more details.
             */
            phoneCam.setRecordingHint(true);



            /*
             * Demonstrate how to lock the camera hardware to sending frames at 30FPS, if it supports that
             */
            for (OpenCvInternalCamera.FrameTimingRange r : phoneCam.getFrameTimingRangesSupportedByHardware())
            {
                if(r.max == 30 && r.min == 30)
                {
                    phoneCam.setHardwareFrameTimingRange(r);
                    break;
                }
            }

            waitForStart();

            while (opModeIsActive())
            {
                telemetry.addData("recording", "");
                telemetry.update();
                sleep(100);
            }
        }

        class UselessColorBoxDrawingPipeline extends OpenCvPipeline
        {
            Scalar color;

            UselessColorBoxDrawingPipeline(Scalar color)
            {
                this.color = color;
            }

            @Override
            public Mat processFrame(Mat input)
            {
                Imgproc.rectangle(
                        input,
                        new Point(
                                input.cols()/4,
                                input.rows()/4),
                        new Point(
                                input.cols()*(3f/4f),
                                input.rows()*(3f/4f)),
                        color, 4);
                return input;
            }
        }
        //tutorialspoints.com/java_dip/color_space_conversion.htm based off of this website
        class convertColorspacePipeline extends OpenCvPipeline
        {

            public Mat processFrame(Mat input){

                //System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

                Mat mat = new Mat(input.rows(), input.cols(), CvType.CV_8UC3);

                Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

//                //byte[] data1 = new byte[mat1.rows() * mat1.cols() * (int) (mat1.elemSize())];
//                mat1.get(0, 0, data1);
//                BufferedImage image1 = new BufferedImage(mat1.cols(), mat1.rows(), 5);
//                image1.getRaster().setDataElements(0, 0, mat1.cols(), mat1.rows(), data1);
//
//                File ouptut = new File("hsv.jpg");
//                ImageIO.write(image1, "jpg", ouptut);
             return mat;
            }
        }
    }

