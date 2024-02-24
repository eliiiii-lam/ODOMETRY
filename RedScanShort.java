package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@Autonomous (name = "RedScanShort", group = "AUTO")
public class RedScanShort extends LinearOpMode {

    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;

    DcMotor elbow;
    DcMotor elbow2;


    Servo clawL;
    Servo clawR;

    Servo wrist;

    OpenCvCamera webcam;

    private IMU imu = null;

    private ElapsedTime runtime = new ElapsedTime();


    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.77952;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)




    // Define motors and servos




    RedSightPipeline pipeline = new RedSightPipeline(telemetry);

    @Override
    public void runOpMode() {
        //initialize robot hardware
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0,0,0);

        fr = hardwareMap.get(DcMotor.class, "frontEncoder");
        bl = hardwareMap.get(DcMotor.class, "leftEncoder");
        br = hardwareMap.get(DcMotor.class, "rightEncoder");
        fl = hardwareMap.get(DcMotor.class, "fL");


        elbow = hardwareMap.dcMotor.get("elbow");
        elbow2 = hardwareMap.dcMotor.get("elbow2");

        wrist = hardwareMap.servo.get("wrist");

        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");





        clawL.setPosition(0.4);
        clawR.setPosition(0);





        elbow.setDirection(DcMotor.Direction.REVERSE);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);



        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".



        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);













        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);










        //FOR THE WEBCAM
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * Webcam stream goes to RC phone
         */


        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously which allows faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                //320px x 340px
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                /*
                 * Specify the image processing pipeline we wish to invoke upon receipt
                 * of a frame from the camera. Note that switching pipelines on-the-fly
                 * (while a streaming session is in flight) *IS* supported.
                 */

                webcam.setPipeline(pipeline);

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("errorCode", errorCode);
            }
        });
        // Tell telemetry to update faster than the default 250ms period :)
        //telemetry.setMsTransmissionInterval(20);

        // telemetry.addLine("Waiting for start");
        // telemetry.update();

        //Wait for the user to press start on the Driver Station

        waitForStart();

        //Manages Telemetry and stopping the stream
        while (opModeIsActive()) {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            switch (pipeline.getAnalysis()) {
                case LEFT:
                    Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)))
                            .lineToLinearHeading(new Pose2d(-50,-11.5,Math.toRadians(180)))
                            .build();
                    Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                            .lineToSplineHeading(new Pose2d(-53,-15,Math.toRadians(90)))
                            .build();

                    Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                            .forward(98)
                            .build();

                    Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                                    .strafeRight(22)
                                            .build();

                    drive.followTrajectory(traj1);
                    wrist.setPosition(0.19);
                    sleep(1000);
                    clawR.setPosition(0.6);
                    sleep(500);
                    wrist.setPosition(0.5);
                    sleep(200);
                    drive.followTrajectory(traj2);
                    wrist.setPosition(0.35);
                    sleep(150);
                    drive.followTrajectory(traj3);
                    sleep(300);
                    drive.followTrajectory(traj4);
                    wrist.setPosition(0.8);
                    sleep(300);
                    drive.turn(Math.toRadians(0));
                    elbowDrive(0.8,90,3);
                    clawL.setPosition(0.05);








                    break;
                case CENTER:
                    Trajectory traj1C = drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)))
                            .lineToLinearHeading(new Pose2d(-51.5,-6,Math.toRadians(180)))
                            .build();
                    Trajectory trajidk = drive.trajectoryBuilder(traj1C.end())
                            .lineToLinearHeading(new Pose2d(-53,-4,Math.toRadians(90)))
                            .build();
                    Trajectory traj2C = drive.trajectoryBuilder(trajidk.end())
                                    .forward(86)
                                            .build();

                    Trajectory traj3C = drive.trajectoryBuilder(traj2C.end())
                                    .strafeRight(29)
                                            .build();
                    Trajectory traj4c = drive.trajectoryBuilder(traj2C.end())
                                    .lineToConstantHeading(new Vector2d(0,-20))
                                            .build();


                    drive.followTrajectory(traj1C);
                    wrist.setPosition(0.19);
                    sleep(1000);
                    clawR.setPosition(0.6);
                    sleep(500);
                    wrist.setPosition(0.5);
                    sleep(200);
                    drive.followTrajectory(trajidk);
                    sleep(300);
                    wrist.setPosition(0.8);
                    clawR.setPosition(0.4);
                    drive.followTrajectory(traj2C);
                    drive.followTrajectory(traj3C);
                    //drive.followTrajectory(traj3C);

                    sleep(300);
                    wrist.setPosition(0.8);
                    elbowDrive(0.8,90,3);
                    clawL.setPosition(0.05);
                    sleep(3000);



                    break;
                case RIGHT:
                    Trajectory traj1R = drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)))
                            .lineToLinearHeading(new Pose2d(-25,-2.1,Math.toRadians(270)))
                            .build();
                    Trajectory trajheheheha = drive.trajectoryBuilder(traj1R.end())
                            .forward(12)
                            .build();
                    Trajectory traj2R = drive.trajectoryBuilder(trajheheheha.end())
                            .lineToLinearHeading(new Pose2d(-52,0,Math.toRadians(90)))
                            .build();
                    Trajectory traj3R = drive.trajectoryBuilder(traj2R.end())
                            .forward(83)
                            .build();
                    Trajectory traj4R = drive.trajectoryBuilder(traj3R.end())
                                    .strafeRight(35)
                                            .build();
                    drive.followTrajectory(traj1R);
                    wrist.setPosition(0.19);
                    sleep(1000);
                    clawR.setPosition(0.6);
                    sleep(500);
                    wrist.setPosition(0.5);
                    sleep(200);
                    drive.followTrajectory(trajheheheha);
                    sleep(200);
                    drive.followTrajectory(traj2R);
                    drive.followTrajectory(traj3R);
                    drive.followTrajectory(traj4R);
                    clawR.setPosition(0.4);
                    sleep(300);
                    wrist.setPosition(0.8);
                    elbowDrive(0.8,90,3);
                    clawL.setPosition(0.05);
                    sleep(3000);



                    break;
            }

            //reminder to use the KNO3 auto transitioner once this code is working

            webcam.stopStreaming();
            webcam.closeCameraDevice();
            break;
        }
    }

    public void elbowDrive(double speed, double elbowInches ,double timeoutS) {
        int newelbowtarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newelbowtarget = elbow.getCurrentPosition() + (int) (elbowInches * COUNTS_PER_INCH);
            elbow.setTargetPosition(newelbowtarget);

            // Turn On RUN_TO_POSITION
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            elbow.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (elbow.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", "%7d", newelbowtarget);
                telemetry.addData("Currently at", "at %7d",
                        elbow.getCurrentPosition());
                telemetry.update();
            }

            elbow.setPower(0);

            // Turn off RUN_TO_POSITION
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
