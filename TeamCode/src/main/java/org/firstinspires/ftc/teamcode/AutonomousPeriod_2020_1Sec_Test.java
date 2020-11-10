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
import com.qualcomm.robotcore.util.Range;


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

@Autonomous(name="AutonomousPeriod_1Sec_Test", group="Test2020")
//@Disabled
public class AutonomousPeriod_2020_1Sec_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    =0.78 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.5;



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
        /*BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);*/

        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData(">", "Press Start for 1 second test" );
        telemetry.update();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            telemetry.addData("Starttime is ", "%5.2f", runtime.time());
            telemetry.update();
            int icounter = 0;
           // while ( icounter < 1) {
                // Setup a variable for each drive wheel to save power level for telemetry
                encoderDrive(0.5, 20.0, 20.0, 20.0, 20.0, 2.5);
          //      encoderDrive(0.1, -20.0, -20.0, -20.0, -20.0, 2.5);
            encoderDrive(0.1, 10.0, 10.0, 10.0, 10.0, 2.5);
             //   icounter++;
            //}

            telemetry.addData("Endtime is ", "%5.2f", runtime.time());
           // telemetry.addData("icounter is ", "%2.0f", icounter);
            telemetry.update();
            sleep(1000);
            break;
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


