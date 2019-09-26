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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="Landing Zone NoDrop", group="Autonomous Opmode")
public class AutoOpMode_Linear3 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    //for the left wheel motor
    private DcMotor rightDrive = null;
    //for the right wheel motor
    private DcMotor climber = null;
    //for the rack and pinion motor
    private DcMotor scooper = null;
    private Servo arm = null;
    private Servo arm2 = null;
    private Servo flag = null;
    private DigitalChannel topTouch;
    private DigitalChannel bottomTouch;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //maps left motor to left wheel
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        //maps right motor onto right wheel
        climber = hardwareMap.get(DcMotor.class, "climber");
        //maps the climber motor onto the rack and pinion
        scooper = hardwareMap.get(DcMotor.class, "scooper");
        arm = hardwareMap.get(Servo.class, "arm");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        flag = hardwareMap.get(Servo.class, "flag");
        // get a reference to our digitalTouch object.
        topTouch = hardwareMap.get(DigitalChannel.class, "topTouch");
        bottomTouch = hardwareMap.get(DigitalChannel.class, "bottomTouch");

        // set the digital channel to input.
        bottomTouch.setMode(DigitalChannel.Mode.INPUT);
        topTouch.setMode(DigitalChannel.Mode.INPUT);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        climber.setDirection(DcMotor.Direction.FORWARD);
        scooper.setDirection(DcMotor.Direction.FORWARD);
        //makes the rack and pinion move up

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        arm.setPosition(.86);
        arm2.setPosition(.14);
        // run until the end of the match (driver presses STOP)

//        while (topTouch.getState()) { //bring climber down
//            climber.setPower(-.5);
//        }
//        climber.setPower(0);
//
//        rightDrive.setPower(.5); //reverse
//        leftDrive.setPower(.5);
//        runtime.reset();
//        while (runtime.seconds() < .64){}
//
//        rightDrive.setPower(0); //pause
//        leftDrive.setPower(0);
//        runtime.reset();
//        while (runtime.seconds() < .4){}
//
//        rightDrive.setPower(-1); //left
//        leftDrive.setPower(1);
//        runtime.reset();
//        while (runtime.seconds() < 1.27){}

        rightDrive.setPower(-1); //forward
        leftDrive.setPower(-1);
        runtime.reset();
        while (runtime.seconds() < 2.25){}

        leftDrive.setPower(0); //flag
        rightDrive.setPower(0);
        scooper.setPower(-1);
        runtime.reset();
        while(runtime.seconds() < 1){}

        scooper.setPower(0);

//        leftDrive.setPower(-1); //right
//        rightDrive.setPower(1);
//        scooper.setPower(0);
//        runtime.reset();
//        while (runtime.seconds() < 1.75){}
//
//        leftDrive.setPower(-1); //forward
//        rightDrive.setPower(-1);
//        runtime.reset();
//        while (runtime.seconds() < 4.5) {}

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
