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


@Autonomous(name = "Red Side Auto Bottom", group = "Sensor")
public class RedSideAuto extends LinearOpMode {
    
    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
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
        robot.init(hardwareMap);
        robot.claw.setPosition(1.0);
        
        sleep(200);
        FrontDrive(0.5);
        sleep(1000);
        CancelPowerRobot();
         /*spin(0.4);
         sleep(1000);
         CancelPowerRobot();
         sleep(100000);*/
        
        /*robot.arm.setPower(1.0);
        SlideRight(0.3);
        sleep(3200);
        CancelPowerRobot();
        sleep(2400);
        robot.arm.setPower(1.0);
        sleep(100);
        
        sleep(100);
        sleep(950);
        robot.arm.setPower(0.0);
        sleep(100);
        FrontDrive(0.3);
        sleep(420);
        CancelPowerRobot();
        sleep(100);
        robot.arm.setPower(-1.0);
        sleep(600);
        robot.arm.setPower(0.0);
        sleep(200);
        robot.claw.setPosition(0.0);
        sleep(100);
        RearDrive(0.3);
        sleep(450);
        CancelPowerRobot();
        robot.arm.setPower(-1.0);
        sleep(100);
        SlideLeft(0.45);
        sleep(3200);
        CancelPowerRobot();
        FrontDrive(0.3);
        sleep(2000);
        CancelPowerRobot();*/
        
        /*sleep(2000);
        
        robot.arm.setPower(1.0);
        sleep(1200);
        robot.arm.setPower(0.32);
                robot.claw.setPosition(1.0);

        sleep(500);
        
        SlideLeft(0.47);
        sleep(600);// 600 before
        CancelPowerRobot();
        sleep(5000);
        int loc = 99;
        int r = robot.color.red();
        int g = robot.color.green();
        int b = robot.color.blue();
        if(r > b && r > g){
            loc = 1;
        }
        if(g > b && g > r){
            loc = 2;
        }
        if(b > g && b > r){
            loc = 3;
        }
        
        telemetry.addData("r", r);
        telemetry.addData("b", b);
        telemetry.addData("g", g);
        sleep(4000);
        robot.claw.setPosition(1.0);
        sleep(100);
        robot.claw.setPosition(1.0);
        SlideLeft(0.6);
        sleep(860);
        CancelPowerRobot();
        sleep(300);
        RearDrive(0.35);
        sleep(240);
        CancelPowerRobot();
        sleep(200); 
        robot.arm.setPower(-0.5);
        sleep(500);
        robot.arm.setPower(0.0);
        robot.claw.setPosition(0.0);
        FrontDrive(0.3);
        sleep(160);
        CancelPowerRobot();
        telemetry.addData("Location", loc);
        telemetry.update();
        sleep(200);
        SlideRight(0.4);
        sleep(750);
        if(loc == 3){
            FrontDrive(0.5);
            sleep(650);
        }
        if(loc == 1){
            RearDrive(0.5);
            sleep(700);
        }
        // telemetry.addData("Sensor1",robot.iL.getDistance(DistanceUnit.INCH));
        // telemetry.update();
        // sleep(3000);
        // long start = System.currentTimeMillis();
        // SlideRight(0.25);
        // while(System.currentTimeMillis()-start<= 5000){
        //     double sense=robot.iR.getDistance(DistanceUnit.INCH);
        //     telemetry.addData("Sensor2",sense);
        //     telemetry.update();
            
        //     if(sense<12){
        //         CancelPowerRobot();
        //         sleep(10000);
       
        //         SlideLeft(0.25);
                
        //         while(robot.iL.getDistance(DistanceUnit.INCH)>100){
        //             ;
        //         }
                
        //         CancelPowerRobot();
        //         sleep(5000);
        //         break;
        //     }
        // }*/
       
        
        
        
        
        
    }

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
    public void CancelPowerRobot(){
        double power=0.0;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
    }
    
    public void robotsleep(long time){
        sleep(time);
    }
}





























