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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutonomousPeriod_EncoderTest", group="Test2020")
//@Disabled
public class AutonomousPeriod_2020_EncoderTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.78 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.5;
    private DistanceSensor sensorRange;



    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        BackLeftDrive  = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeftDrive  = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //This is very important to keep left drive to forward and right drive to REVERSE
       /* BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);*/

        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);


        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       /* BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        BackLeftDrive.setTargetPosition(2500);
        BackRightDrive.setTargetPosition(2500);
        FrontLeftDrive.setTargetPosition(2500);
        FrontRightDrive.setTargetPosition(2500);

        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        telemetry.addData("Mode", "waiting");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        BackLeftDrive.setPower(0.247);
        BackRightDrive.setPower(0.25);
        FrontLeftDrive.setPower(0.247);
        FrontRightDrive.setPower(0.25);

        while (opModeIsActive() && FrontLeftDrive.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left", FrontLeftDrive.getCurrentPosition() + "  busy=" + FrontLeftDrive.isBusy());
            telemetry.addData("encoder-fwd-right", FrontRightDrive.getCurrentPosition() + "  busy=" + FrontRightDrive.isBusy());
            telemetry.addData("encoder-fwd-left", BackLeftDrive.getCurrentPosition() + "  busy=" + BackLeftDrive.isBusy());
            telemetry.addData("encoder-fwd-right", BackRightDrive.getCurrentPosition() + "  busy=" + BackRightDrive.isBusy());

            telemetry.update();
            idle();

            // you can use this as a regular DistanceSensor.
         /*   sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

            // you can also cast this to a Rev2mDistanceSensor if you want to use added
            // methods associated with the Rev2mDistanceSensor class.

            if ( sensorRange.getDistance(DistanceUnit.INCH) <= 55)
            {
                idle();
                break;
            }
*/
        }


        telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));


        telemetry.update();
        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        FrontLeftDrive.setPower(0.0);
        FrontRightDrive.setPower(0.0);
        BackLeftDrive.setPower(0.0);
        BackRightDrive.setPower(0.0);
        // wait 5 sec to you can observe the final encoder position.


 /*

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 1)
        {
            telemetry.addData("encoder-fwd-left-end", FrontLeftDrive.getCurrentPosition());
            telemetry.addData("encoder-fwd-right-end", FrontRightDrive.getCurrentPosition());
            telemetry.addData("encoder-fwd-left-end", BackLeftDrive.getCurrentPosition());
            telemetry.addData("encoder-fwd-right-end", BackRightDrive.getCurrentPosition());
            telemetry.update();
            idle();
        }
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BackLeftDrive.setTargetPosition(0);
        BackRightDrive.setTargetPosition(0);
        FrontLeftDrive.setTargetPosition(0);
        FrontRightDrive.setTargetPosition(0);

        // Power sign matters again as we are running without encoder.
        BackLeftDrive.setPower(-0.247);
        BackRightDrive.setPower(-0.25);
        FrontLeftDrive.setPower(-0.247);
        FrontRightDrive.setPower(-0.25);

        while (opModeIsActive() && FrontLeftDrive.getCurrentPosition() > FrontLeftDrive.getTargetPosition())
        {
            telemetry.addData("encoder-back-left", FrontLeftDrive.getCurrentPosition());
            telemetry.addData("encoder-back-right", FrontRightDrive.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // set motor power to zero to stop motors.

        FrontLeftDrive.setPower(0.0);
        FrontRightDrive.setPower(0.0);
        BackLeftDrive.setPower(0.0);
        BackRightDrive.setPower(0.0);

        resetStartTime();
*/
        while (opModeIsActive() && getRuntime() < 5)
        {

            if ( sensorRange.getDistance(DistanceUnit.INCH) <= 55)
            {
                telemetry.addData("encoder-fwd-left-end", FrontLeftDrive.getCurrentPosition());
                telemetry.addData("encoder-fwd-right-end", FrontRightDrive.getCurrentPosition());
                telemetry.addData("encoder-fwd-left-end", BackLeftDrive.getCurrentPosition());
                telemetry.addData("encoder-fwd-right-end", BackRightDrive.getCurrentPosition());
                telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                idle();
            }
        }




    }


public void encoderDrive(double speed, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = FrontLeftDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newFrontRightTarget = FrontRightDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newBackLeftTarget = BackLeftDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newBackRightTarget = BackRightDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            FrontLeftDrive.setTargetPosition(newFrontLeftTarget);
            FrontRightDrive.setTargetPosition(newFrontRightTarget);
            BackLeftDrive.setTargetPosition(newBackLeftTarget);
            BackRightDrive.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
          //  runtime.reset();
            FrontLeftDrive.setPower(Math.abs(speed));
            FrontRightDrive.setPower(Math.abs(speed));
            BackLeftDrive.setPower(Math.abs(speed));
            BackRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
              (runtime.seconds() < timeoutS)
                 &&  (BackLeftDrive.isBusy() && BackRightDrive.isBusy()))
                    {

                // Display it for the driver.
               telemetry.addData("Path1",  "Running to %7d :%7d", newBackLeftTarget,  newBackRightTarget);
               telemetry.addData("Path2",  "Running at %7d :%7d",
                      BackLeftDrive.getCurrentPosition(),
                       BackRightDrive.getCurrentPosition());
                telemetry.update();
            }




            // Stop all motion;
            FrontLeftDrive.setPower(0);
            FrontRightDrive.setPower(0);
            BackLeftDrive.setPower(0);
            BackRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(25);   // optional pause after each move
        }
    }
}


