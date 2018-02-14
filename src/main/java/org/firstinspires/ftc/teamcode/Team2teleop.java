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

@TeleOp(name="Team2_telop", group="Linear Opmode")
//@Disabled
public class Team2teleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;

    //I got rid of rearRightServo and rearLeftServo

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elevatorDrive = null;
    private DcMotor linearDrive = null;
    private Servo GrabLift = null;
    public Servo GrabMove = null;
    private Servo fingerservo = null;
    private Servo wristservo = null;
    private CRServo UpDown1 = null;
    private CRServo UpDown2 = null;


    double minServoPosition =.02;
    double maxServoPosition =1;


    @Override


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //rightDrive = hardwareMap.get(DcMotor.class, "right_drive");






        UpDown1 = hardwareMap.get(CRServo.class, "up_down1");
        UpDown2 = hardwareMap.get(CRServo.class, "up_down2");
        fingerservo = hardwareMap.get(Servo.class, "finger_servo");
        wristservo = hardwareMap.get(Servo.class, "wrist_servo");
        GrabMove = hardwareMap.get(Servo.class, "move_servo");
        GrabLift = hardwareMap.get(Servo.class, "lift_servo");
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        elevatorDrive = hardwareMap.get(DcMotor.class, "elevator_drive");
        linearDrive = hardwareMap.get(DcMotor.class, "linear_drive");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);

        double oppspeed = -.5;
        double nospeed = 0;
        double halfspeed = .5;
        final double CLAW_SPEED = 0.02;
        double leftPower;
        double rightPower;
        double elevatorPower;
        double linearPower;
        double grabMovePosition;
        double grabLiftPosition;
        double wristServoPosition;
        double fingerServoPosition;
        double upDownPower1=0;
        double upDownPower2=0;



        /*
        GrabLift.setPosition(minServoPosition);
        GrabMove.setPosition(minServoPosition);
        wristservo.setPosition(minServoPosition);
        fingerservo.setPosition(maxServoPosition);
        UpDown1.setPosition(minServoPosition);
        UpDown2.setPosition(minServoPosition);
            */


        grabLiftPosition=0;
        grabMovePosition=0;
        fingerServoPosition=1;
        wristServoPosition=0;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        linearDrive.setDirection(DcMotor.Direction.FORWARD);
        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        GrabLift.setDirection(Servo.Direction.FORWARD);
        fingerservo.setDirection(Servo.Direction.FORWARD);
        wristservo.setDirection(Servo.Direction.REVERSE);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Setup a variable for each drive wheel to save power level for telemetry



            //5 Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.

            leftPower  = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y ;
            elevatorPower = gamepad2.left_stick_y;
            linearPower = -gamepad2.right_stick_y ;



            //servos to pick up blocks
            if (gamepad2.x) {
                grabMovePosition=(GrabMove.getPosition() - CLAW_SPEED);
                grabLiftPosition=(GrabLift.getPosition()+CLAW_SPEED);
            }
            else if (gamepad2.y) {
                grabMovePosition=(GrabMove.getPosition() + CLAW_SPEED);
                grabLiftPosition=(GrabLift.getPosition()-CLAW_SPEED);

                }

            //servos to move finger servo
            if (gamepad2.a)

                fingerServoPosition=(fingerservo.getPosition()+CLAW_SPEED);
                //servo1.setPosition(servo1.getPosition()+CLAW_SPEED);

            else if (gamepad2.b)
                fingerServoPosition=(fingerservo.getPosition()-CLAW_SPEED);

            //servos to move wrist servo
            if (gamepad2.dpad_down)
                wristServoPosition=(wristservo.getPosition() + CLAW_SPEED);
                //servo1.setPosition(servo1.getPosition()+CLAW_SPEED);

            else if (gamepad2.dpad_up)
                wristServoPosition=(wristservo.getPosition() - CLAW_SPEED);

            //updown servos below;
            if (gamepad1.dpad_up) {
                upDownPower1=(halfspeed);
                upDownPower2=(oppspeed);
            }
            else if (gamepad1.dpad_down) {
                upDownPower1 = (oppspeed);
                upDownPower2 = (halfspeed);
            }
            else if (gamepad1.dpad_left) {
                upDownPower1 = (nospeed);
                upDownPower2 = (nospeed);
            }

            // Send calculated power to wheels

            GrabMove.setPosition(Range.clip(grabMovePosition,0,1));
            GrabLift.setPosition((Range.clip(grabLiftPosition, 0, 1)));
            fingerservo.setPosition(Range.clip(fingerServoPosition, 0, 1 ));
            wristservo.setPosition((Range.clip(wristServoPosition, 0, 1 )));



            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            elevatorDrive.setPower(elevatorPower);
            linearDrive.setPower(linearPower);
            UpDown1.setPower(upDownPower1);
            UpDown2.setPower(upDownPower2);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
