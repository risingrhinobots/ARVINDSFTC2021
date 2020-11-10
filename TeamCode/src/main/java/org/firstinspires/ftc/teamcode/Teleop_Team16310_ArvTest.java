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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Teleop-Team16310 Arv", group = "Test")
@Disabled
public class Teleop_Team16310_ArvTest extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.45;     // Maximum rotational position
    static final double MIN_POS     =  0.12;     // Minimum rotational position
    static final double MAX_FWD_POS     =  0.75;     // Maximum rotational position
    static final double MIN_FWD_POS     =  0;     // Minimum rotational position
    static final double MAX_FWD_POS_BRK_UPPER     =  0.7;     // Maximum rotational position
    static final double MIN_FWD_POS_BRK_UPPER     =  0;     // Minimum rotational position
    static final double MAX_FWD_POS_BRK_LOWER     =  0.25;     // Maximum rotational position
    static final double MIN_FWD_POS_BRK_LOWER     =  0;     // Minimum rotational position
    // Define class members
    Servo   servoup;
    Servo   servobrick;
    Servo   servoforward;
    Servo   servolower;
    Servo   servoupper;

//    double armforward1= (MAX_POS - MIN_POS) / 2;;
//    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double armforward1= 0.08;  // controls the positioning of the arm controlled by servoforward
    double  position = 0.22; // controls the positioning of the arm controlled by the servoup
    double  armPosition1; // controls the positioning of the servo brick controlled by servobrick
    double  brickholderupper; // controls the positioning of the servo brick controlled by servoupper
    double  brickholderlower; // controls the positioning of the servo brick controlled by servolower

    double startingServoUpPosx = 0.22;
    double startingServoUpPosy = 0.38;
    double startingServoForwardPos = 0.08;


    boolean rampUp = true;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    double  MIN_POSITION1 = -0.2, MAX_POSITION1 = 0.6;

    @Override
    public void runOpMode() {

        // define the motors
        leftDrive  = hardwareMap.get(DcMotor.class, "Backleft");
        rightDrive = hardwareMap.get(DcMotor.class, "Backright");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //This is very important to keep left drive to forward and right drive to REVERSE
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servoup = hardwareMap.get(Servo.class, "ServoUp");
        servobrick = hardwareMap.get(Servo.class, "Brick");
        servoforward = hardwareMap.get(Servo.class, "ServoForward");
        servolower = hardwareMap.get(Servo.class, "BrickL");
        servoupper = hardwareMap.get(Servo.class, "BrickR");


        servoforward.setPosition(startingServoForwardPos);
        servoup.setPosition(startingServoUpPosx);
        servolower.setPosition(0.2);
        servoupper.setPosition(0.5);

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //Please note it is very important to note that DRIVE is set to positive and

            double drive2 =  gamepad1.left_stick_y;
            double turn2  =  gamepad1.right_stick_x;

            // Display the current value
            telemetry.addData("game pad2 left stick Position", "%5.2f", drive2);
            telemetry.addData("game pad2 right stick Position", "%5.2f", turn2);
            telemetry.update();


            double drive =  gamepad1.left_stick_y;
            double turn  =  -gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -0.75, 0.75) ;
            rightPower   = Range.clip(drive - turn, -0.75, 0.75) ;



            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);


            if (gamepad2.x) {
                armforward1= startingServoForwardPos;
                position = startingServoUpPosx; // Start at halfway position
                servoforward.setPosition(armforward1);
                servoup.setPosition(position);
            }

            if (gamepad2.y) {
                armforward1= startingServoForwardPos;
                position = startingServoUpPosy; // Start at halfway position
                servoforward.setPosition(armforward1);
                servoup.setPosition(position);
            }

                // slew the servo, according to the rampUp (direction) variable.
            if (gamepad2.a) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT;
                if (position >= MAX_POS) {
                    position = MAX_POS;

                }
            } else if (gamepad2.b) {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT;
                if (position <= MIN_POS) {
                    position = MIN_POS;

                }
            }
            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test.");

            // Set the servo to the new position and pause;
            servoup.setPosition(position);
            sleep(CYCLE_MS);
            idle();

            if (gamepad2.dpad_right  ) {
                // Keep stepping up until we hit the max value.
                armforward1 += INCREMENT;
                if (armforward1 >= MAX_FWD_POS) {
                    armforward1 = MAX_FWD_POS;

                }
            } else if (gamepad2.dpad_left) {
                // Keep stepping down until we hit the min value.
                armforward1 -= INCREMENT;
                if (armforward1 <= MIN_FWD_POS) {
                    armforward1 = MIN_FWD_POS;

                }
            }


            // Display the current value
            telemetry.addData("Servo Forward Position", "%5.2f", armforward1);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the servo to the new position and pause;
            servoforward.setPosition(armforward1);
            sleep(CYCLE_MS);
            idle();

/*
            if (gamepad2.dpad_up  ) {
                // Keep stepping up until we hit the max value.
                brickholderlower += INCREMENT;
                if (brickholderlower >= MAX_FWD_POS_BRK_LOWER) {
                    brickholderlower = MAX_FWD_POS_BRK_LOWER;
                }

            } else if (gamepad2.dpad_down) {
                // Keep stepping down until we hit the min value.
                brickholderlower -= INCREMENT;
                if (brickholderlower <= MIN_FWD_POS_BRK_LOWER) {
                    brickholderlower = MIN_FWD_POS_BRK_LOWER;
                }

            }
*/
            if (gamepad2.dpad_up  ) {
                // Keep stepping up until we hit the max value.
                brickholderupper += INCREMENT;
                if (brickholderupper >= MAX_FWD_POS_BRK_UPPER) {
                    brickholderupper = MAX_FWD_POS_BRK_UPPER;
                }

                brickholderlower += INCREMENT;
                if (brickholderlower >= MAX_FWD_POS_BRK_LOWER) {
                    brickholderlower = MAX_FWD_POS_BRK_LOWER;
                }


            } else if (gamepad2.dpad_down) {
                // Keep stepping down until we hit the min value.
                brickholderupper -= INCREMENT;
                if (brickholderupper <= MIN_FWD_POS_BRK_UPPER) {
                    brickholderupper = MIN_FWD_POS_BRK_UPPER;
                }

                brickholderlower -= INCREMENT;
                if (brickholderlower <= MIN_FWD_POS_BRK_LOWER) {
                    brickholderlower = MIN_FWD_POS_BRK_LOWER;
                }

            }

            servoupper.setPosition(brickholderupper);
            servolower.setPosition(brickholderlower);
            sleep(CYCLE_MS);
            idle();
;
            // Display the current value
            telemetry.addData("Servo Brick lower ", "%5.2f", brickholderlower);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();


            // Display the current value
            telemetry.addData("Servo Brick lower ", "%5.2f", brickholderlower);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();



            if (gamepad1.dpad_up  ) {
                // Keep stepping up until we hit the max value.
                armPosition1 += 0.1;
                if (armPosition1 >= MAX_POSITION1) {
                    armPosition1 = MAX_POSITION1;

                }
            } else if (gamepad1.dpad_down) {
                // Keep stepping down until we hit the min value.
                armPosition1 -= 0.1;
                if (armPosition1 <= MIN_POSITION1) {
                    armPosition1 = MIN_POSITION1;
                }
            }
                     // Display the current value
            telemetry.addData("Servo Brick Position", "%5.2f", armPosition1);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the servo to the new position and pause;
            servobrick.setPosition(armPosition1);

            // Display the current value
            telemetry.addData("Servo Brick Position", "%5.2f", armPosition1);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

        }
       // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

