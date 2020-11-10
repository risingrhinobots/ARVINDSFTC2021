/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Atonomus_Foundation_VUF", group = "Test")
@Disabled
public class Atonomus_Foundation_VUF extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot_Test robot = new HardwarePushbot_Test();   // Use a Pushbot's hardware
    Autonomus_Mechanum_Wheel_Test2_GC drive = new Autonomus_Mechanum_Wheel_Test2_GC();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "ASMsUFL/////AAABmfXWIwtIlUd+nWgogvcNtm58RzGLx1JFvc1ib3D2jnqPKWmB4f6junaSWGKJ029K/4LooVMWYBrWAvtLmE26GFQGGae6OxKm0FIkDjrpOwtvJoAysRG0eYZUk+D6D7YX48T+B0ycpXAZKg10ibuMAsa6lhLh+/KmN6MYQefuSa4lgmJT3oEqzR1I8KPZntdA/bpVxfVnV9S36fqkMicN5ATLhS0Qv9pH+zuPM8M0PS5Fn6riBQY7zne+v5bNl81rLE83VCzltUI6a3vfqX9IOqQNGGu0HW4Uzvlfv5kh5wb4ot2niaK1nO1ZswLMu02Hfru2EEXpHDWkBwnBiUnchih+UE9tFGAsmlXR2LwLJd/f";
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

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
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition());
        robot.backLeft.getCurrentPosition();
        robot.backRight.getCurrentPosition();
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));



        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        //Turn Flash light on
        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);

        //Set zoom of camera
        //com.vuforia.CameraDevice.getInstance().setField("opti-zoom", "opti-zoom-on");
        //com.vuforia.CameraDevice.getInstance().setField("zoom", "15");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        targetsSkyStone.activate();

        //robot.up.setPosition(0.6);

        encoderDrive1(0.2,-10,10,10,-10.3,15.0, allTrackables);
        encoderDrive1(0.2,15,15,15,15,15.0, allTrackables);
        robot.BrickL.setPosition(0.01);
        robot.BrickR.setPosition(0.99);
        sleep(2000);
        encoderDrive1(0.2,-15,-15,-15,-15,9.0,allTrackables);
        robot.BrickL.setPosition(0.5);
        robot.BrickR.setPosition(0.5);
        encoderDrive1(0.2,20,-20,-20,20,15.0, allTrackables);

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive1(double speed, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches, double timeoutS,List<VuforiaTrackable> allTrackables) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.backRight.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.backLeft.isBusy() && robot.backRight.isBusy()) && robot.frontLeft.isBusy() && robot.backRight.isBusy() ) {

                if (Skystone_Visible(allTrackables)) {
                    robot.frontLeft.setPower(0);
                    robot.frontRight.setPower(0);
                    robot.backLeft.setPower(0);
                    robot.backRight.setPower(0);
                    encoderDrive2(0.1,-5,-5.0,-5.0,-5.0,5.0);
                    encoderDrive2(0.2,-5.0,5.0,5.0,-5.0,5.0);
                    sleep(2000);
                    robot.Claw.setPosition(0.1);
                    sleep(2000);
                    robot.up.setPosition(0.25);

                }

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.backLeft.getCurrentPosition(),
                        robot.backRight.getCurrentPosition());
                robot.frontLeft.getCurrentPosition();
                robot.frontRight.getCurrentPosition();
                telemetry.update();
            }


            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    private boolean Skystone_Visible(List<VuforiaTrackable> allTrackables) {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;


        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());

                if (trackable.getName().equals("Stone Target")) {
                    telemetry.addLine("Stone Target Is Visible");

                }

                targetVisible = true;

            }

        }
        return targetVisible;
    }


     public void encoderDrive2(double speed, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches, double timeoutS) {
     int newFrontLeftTarget;
     int newFrontRightTarget;
     int newBackLeftTarget;
     int newBackRightTarget;

     // Ensure that the opmode is still active
     if (opModeIsActive()) {

     // Determine new target position, and pass to motor controller
     newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
     newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
     newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
     newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
     robot.frontLeft.setTargetPosition(newFrontLeftTarget);
     robot.frontRight.setTargetPosition(newFrontRightTarget);
     robot.backLeft.setTargetPosition(newBackLeftTarget);
     robot.backRight.setTargetPosition(newBackRightTarget);

     // Turn On RUN_TO_POSITION
     robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


     // reset the timeout time and start motion.
     runtime.reset();
     robot.frontLeft.setPower(Math.abs(speed));
     robot.frontRight.setPower(Math.abs(speed));
     robot.backLeft.setPower(Math.abs(speed));
     robot.backRight.setPower(Math.abs(speed));

     // keep looping while we are still active, and there is time left, and both motors are running.
     // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
     // its target position, the motion will stop.  This is "safer" in the event that the robot will
     // always end the motion as soon as possible.
     // However, if you require that BOTH motors have finished their moves before the robot continues
     // onto the next step, use (isBusy() || isBusy()) in the loop test.

     while (opModeIsActive() &&
     (runtime.seconds() < timeoutS) &&
     (robot.backLeft.isBusy() && robot.backRight.isBusy()) && robot.frontLeft.isBusy() && robot.backRight.isBusy()) {

     // Display it for the driver.
     telemetry.addData("Path1",  "Running to %7d :%7d", newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
     telemetry.addData("Path2",  "Running at %7d :%7d",
     robot.backLeft.getCurrentPosition(),
     robot.backRight.getCurrentPosition());
     robot.frontLeft.getCurrentPosition();
     robot.frontRight.getCurrentPosition();
     telemetry.update();
     }




     // Stop all motion;
     robot.frontLeft.setPower(0);
     robot.frontRight.setPower(0);
     robot.backLeft.setPower(0);
     robot.backRight.setPower(0);

     // Turn off RUN_TO_POSITION
     robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

     sleep(250);   // optional pause after each move
     }
     }

}

