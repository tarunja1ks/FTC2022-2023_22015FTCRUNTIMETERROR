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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.time.LocalDateTime;  


@Autonomous(name = "cycle test", group = "Sensor")
public class CycleTest extends LinearOpMode {
                HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware

    /* Declare OpMode members. */
    // private ElapsedTime     runtime = new ElapsedTime();

    // static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    // static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    // static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    // static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //                                                   (WHEEL_DIAMETER_INCHES * 3.1415);
    // static final double     DRIVE_SPEED             = 0.6;
    // static final double     TURN_SPEED              = 0.5;
   
    @Override
    public void runOpMode(){
        waitForStart();
        robot.init(hardwareMap);// initiliazing everything(motors, sensors,etc.)
        robot.claw.setPosition(0.0); 
        /*
        robot.viper.setPower(1.0);
        sleep(600);
        robot.viper.setPower(0.1);
        
        robot.barm.setPower(0.5);
        sleep(700);
        robot.barm.setPower(0.0);
        
        robot.claw.setPosition(0.8);
        
        robot.bslide.setPower(1.0);
        sleep(1700);
        robot.bslide.setPower(0.0);
        
        robot.claw.setPosition(0.0);*/
        
        
        
               /*robot.box.setPosition(0.75);
       robot.claw.setPosition(0.5);
        sleep(100);
       robot.grab.setPower(-0.4);
       sleep(725);
       robot.grab.setPower(0.0);
       sleep(3000);
       robot.claw.setPosition(0.7);
       sleep(500);
       robot.grab.setPower(0.5);
       sleep(500);
       robot.grab.setPower(0.0);
       sleep(200);
       robot.box.setPosition(0.75);
       sleep(100);
       robot.claw.setPosition(0.1);*/
        
        
        
        
        
    }
  
  
  
  
  
  
  

    // moving functions spin,driving,etc.
    // negative frontright specific to our robot since the wheel moves backwards
    public void spin(double power){
        power=-power;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(-power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(power);
    }
   
    public void FrontDrive(double power){
        power=-power;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
    }
    public void LeftSlantDrive(double power){
        power=-power;
        robot.motorFrontLeft.setPower(0.0);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(0.0);
        robot.motorBackLeft.setPower(power);
    }
    public void RightSlantDrive(double power)
    {
        power = -power;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(0.0);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(0.0);
    }
    
    public void RightSlantRearDrive(double power)
    {
        robot.motorFrontLeft.setPower(0.0);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(0.0);
        robot.motorBackLeft.setPower(power);
    }
    public void LeftSlantRearDrive(double power)
    {
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(0.0);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(0.0);
    }
    public void RearDrive(double power){
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
    }
    public void SlideRight(double power)
    {
        power = -power;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(-power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(-power);
        
    }
    public void SlideLeft(double power)
    {
        power = -power;
        robot.motorFrontLeft.setPower(-power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(power);
        
    }
    // shutting down moving motors
    public void CancelPowerRobot(){
        double power=0.0;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
    }
    // sleeping
    public void robotsleep(long time){
        sleep(time);
    }
}





























