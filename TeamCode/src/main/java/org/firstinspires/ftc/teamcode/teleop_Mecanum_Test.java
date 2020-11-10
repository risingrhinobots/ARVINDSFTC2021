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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="teleop_Mecanum_Test", group="Test")
@Disabled
public class teleop_Mecanum_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private Servo Up;
    private Servo Claw;
    private Servo BrickL;
    private Servo BrickR;

    double UpPosition = 0.3;
    double ClawPosition = 0.3;
    double BrickLPosition = 0.5;
    double BrickRPosition = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FrontLeftDrive  = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeftDrive = hardwareMap.get(DcMotor.class,"BackLeft");
        BackRightDrive = hardwareMap.get(DcMotor.class,"BackRight");
        Up = hardwareMap.get(Servo.class,"Up");
        Claw = hardwareMap.get(Servo.class,"Claw");
        BrickL = hardwareMap.get(Servo.class, "BrickL");
        BrickR = hardwareMap.get(Servo.class,"BrickR");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        Claw.setPosition(0.3);
        Up.setPosition(0.3);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double FrontLeftPower;
            double FrontRightPower;
            double BackLeftPower;
            double BackRightPower;
            double sidePower;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double drive2 = -gamepad1.left_stick_x;


            sidePower = Range.clip(drive2 - 0,-0.60,0.60);
            FrontLeftPower    = Range.clip(drive + turn, -0.60, 0.60);
            FrontRightPower   = Range.clip(drive - turn, -0.60, 0.60);
            BackLeftPower = Range.clip(drive + turn, -0.60, 0.60);
            BackRightPower = Range.clip(drive - turn, -0.60, 0.60);

            if (sidePower != 0){
                FrontLeftDrive.setPower(-sidePower);
                FrontRightDrive.setPower(sidePower);
                BackLeftDrive.setPower(sidePower);
                BackRightDrive.setPower(-sidePower);
            } else {
                FrontLeftDrive.setPower(FrontLeftPower);
                FrontRightDrive.setPower(FrontRightPower);
                BackLeftDrive.setPower(BackLeftPower);
                BackRightDrive.setPower(BackRightPower);
            }

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            /**

             if (gamepad1.dpad_right){
             FrontLeftPower = 0.6;
             FrontRightPower = -0.6;
             BackLeftPower = -0.6;
             BackRightPower = 0.6;

             } else {
             FrontLeftPower = +FrontLeftPower;
             FrontRightPower = +FrontRightPower;
             BackLeftPower = +BackLeftPower;
             BackRightPower = +BackRightPower;
             }

             if (gamepad1.dpad_left){
             FrontLeftPower = -0.6;
             FrontRightPower = 0.6;
             BackLeftPower = 0.6;
             BackRightPower = -0.6;

             } else {
             FrontLeftPower = +FrontLeftPower;
             FrontRightPower = +FrontRightPower;
             BackLeftPower = +BackLeftPower;
             BackRightPower = +BackRightPower;
             }

             */

            // Send calculated power to wheels
            FrontLeftDrive.setPower(FrontLeftPower);
            FrontRightDrive.setPower(FrontRightPower);
            BackLeftDrive.setPower(BackLeftPower);
            BackRightDrive.setPower(BackRightPower);

            if (gamepad2.dpad_left){
                ClawPosition = ClawPosition + 0.1;
                if (ClawPosition >= 1.0){
                    ClawPosition = 0.0;
                }
            } else if(gamepad2.dpad_right){
                ClawPosition = ClawPosition - 0.1;
                if (ClawPosition <= 0.0){
                    ClawPosition = 0.0;
                }
            }

            if (gamepad1.dpad_down){
                BrickLPosition = BrickLPosition - 0.01;
                BrickRPosition = BrickRPosition +0.01;
                if (BrickLPosition <= 0.0){
                    BrickLPosition = 0.0;
                }
                if (BrickRPosition >= 1.0){
                    BrickRPosition = 1.0;
                }
            } else if (gamepad1.dpad_up){
                BrickLPosition = BrickLPosition +0.01;
                BrickRPosition = BrickRPosition - 0.01;
                if ((BrickLPosition  <= 1.0) || (BrickRPosition <= 0.0)){
                    BrickLPosition = 1.0;
                    BrickRPosition = 0.0;
                }
            }

            if (gamepad1.a){
                BrickLPosition = 0.1;
                BrickRPosition = 0.9;
            }

            if (gamepad1.y){
                BrickRPosition = 0.5;
                BrickLPosition = 0.5;
            }

            BrickL.setPosition(BrickLPosition);
            BrickR.setPosition(BrickRPosition);
            sleep(50);
            idle();

            if (gamepad2.a){
                UpPosition = UpPosition - 0.01;
                if (UpPosition >= 0.3){
                    UpPosition = 0.3;
                }
            }

            if (gamepad2.y) {
                UpPosition = UpPosition + 0.01;
                if (UpPosition >= 0.7) {
                    UpPosition = 0.7;
                }
            }

            Up.setPosition(UpPosition);
            Claw.setPosition(ClawPosition);
            sleep(25);
            idle();


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("ServoUp", UpPosition);
            telemetry.addData("Servo Claw",ClawPosition);
            telemetry.addData("BrickL Position", BrickLPosition);
            telemetry.addData("BrickR Position", BrickRPosition);
            telemetry.addData("Motors", "Frontleft (%.2f), Frontright (%.2f), Backleft (%.2f), Backright (%.2f)", FrontLeftPower, FrontRightPower, BackLeftPower,BackRightPower);
            telemetry.update();
        }
    }
}

