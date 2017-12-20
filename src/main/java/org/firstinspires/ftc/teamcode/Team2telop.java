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
import com.qualcomm.robotcore.hardware.CRServo;

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

@TeleOp(name="Team2telop", group="Linear Opmode")
//@Disabled
public class Team2telop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;

    //I got rid of rearRightServo and rearLeftServo
    private Servo GrabLift = null;
    private Servo RelicGrabber = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elevatorDrive = null;
    private DcMotor armDrive = null;
    public Servo GrabMove;
    double servoStartPos = 0.5;


    @Override


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        GrabMove = hardwareMap.get(Servo.class, "moveServo");
        GrabLift = hardwareMap.get(Servo.class, "liftServo");
        RelicGrabber = hardwareMap.get(Servo.class, "RelicServo");
        elevatorDrive = hardwareMap.get(DcMotor.class, "elevatorDrive");
       armDrive = hardwareMap.get (DcMotor.class, "armDrive");






        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);

        double maxspeed = 1;
        double nospeed = 0;
        double halfspeed = .5;
        double servoDelta = 0.1;
        double leftPower;
        double rightPower;
        double elevatorPower;
        double armPower;
        //sets rate to move servo
        double servo1Position;
        double servo2Position;
        double servo3Position;

        servo1Position=(servoStartPos);
        servo2Position=(servoStartPos);
        servo3Position=(servoStartPos);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        GrabLift.setDirection(Servo.Direction.FORWARD);
        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive.setDirection(DcMotor.Direction.FORWARD);
        RelicGrabber.setDirection(Servo.Direction.FORWARD);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Setup a variable for each drive wheel to save power level for telemetry



            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.

            leftPower  = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y ;
            elevatorPower = -gamepad2.left_stick_y;
            armPower = -gamepad2.right_stick_y;


            if (gamepad1.x) {

                servo1Position -= servoDelta;
                servo2Position += servoDelta;}

                //servo1.setPosition(servo1.getPosition()+CLAW_SPEED);

            else if (gamepad1.y) {
                    servo1Position += servoDelta;
                    servo2Position -= servoDelta;

                }


            if (gamepad1.b) {

                servo3Position -= servoDelta; }


            else if (gamepad1.a) {
                servo3Position += servoDelta;

            }

                servo1Position=Range.clip(servo1Position,0,1);
                servo2Position= Range.clip(servo2Position, 0,  1);
                //servo3Position= Range.clip(servo3Position, 0, 1);

                GrabLift.setPosition(servo1Position);
                GrabMove.setPosition(servo2Position);
                RelicGrabber.setPosition(servo3Position);
            // Send calculated power to wheels

            leftDrive.setPower(leftPower);
            rightDrive.setPower(-rightPower);
            elevatorDrive.setPower(elevatorPower);
            armDrive.setPower(armPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
