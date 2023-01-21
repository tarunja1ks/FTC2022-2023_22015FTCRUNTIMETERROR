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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

// import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot{
    /* Public OpMode members. */
    public DcMotor motorFrontLeft   = null;
    public DcMotor  motorBackRight  = null;
    public DcMotor motorFrontRight= null;
    public DcMotor motorBackLeft=null;
    public DcMotor viper = null;
    public Servo claw = null;

    //infared
    public Rev2mDistanceSensor iR= null;
    public Rev2mDistanceSensor iB= null;
    public Rev2mDistanceSensor iL= null;
    public Rev2mDistanceSensor iF= null;
    public ColorSensor color = null;
    
    
    
    // public DcMotor  leftArm     = null;
    // public Servo    leftClaw    = null;
    // public Servo    rightClaw   = null;

    // public static final double MID_SERVO       =  0.5 ;
    // public static final double ARM_UP_POWER    =  0.45 ;
    // public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
   

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorFrontLeft  = hwMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hwMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hwMap.get(DcMotor.class,"motorBackRight");
        motorBackLeft = hwMap.get(DcMotor.class,"motorBackLeft");
        viper = hwMap.get(DcMotor.class, "viper");
        claw = hwMap.get(Servo.class, "claw");
        color = hwMap.get(ColorSensor.class, "color");
        iR=hwMap.get(Rev2mDistanceSensor.class, "InfRight");
        iB=hwMap.get(Rev2mDistanceSensor.class, "InfBack");
        iL=hwMap.get(Rev2mDistanceSensor.class, "InfLeft");

        
        // iF=hwMap.get(Rev2mDistanceSensor.class, "infFront");
        
    }
 }

