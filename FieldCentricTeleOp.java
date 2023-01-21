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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * {@link ConceptTelemetry} illustrates various ways in which telemetry can be
 * transmitted from the robot controller to the driver station. The sample illustrates
 * numeric and text data, formatted output, and optimized evaluation of expensive-to-acquire
 * information. The telemetry {@link Telemetry#log() log} is illustrated by scrolling a poem
 * to the driver station.
 *
 * @see Telemetry
 */
@TeleOp(name = "Field Centric Teleop", group = "Concept")

public class FieldCentricTeleOp extends LinearOpMode  {
    /** keeps track of the line of the poem which is to be emitted next */
    HardwarePushbot robot = new HardwarePushbot();
    public double position=0.0;
    public double lcpos=1.0;
    public boolean lock=false;
    public void runOpMode(){

        robot.init(hardwareMap);
        telemetry.addData("Say", "Running: waiting for controller input");
        telemetry.update();
        

        double left;
        double right;
        double drive;
        double turn;
        double max;
        double scoop_turn = 0.5;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //robot.leftClaw.scaleRange(0.4,0.6);
        // run until the end of the match (driver presses STOP)
        boolean closed = false;
        boolean start = true;
        boolean cycle = false;
        boolean closed2 = false;
        boolean lastDir = false;
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        while (opModeIsActive()) {
            telemetry.addData("closed", closed);
            telemetry.update();
            start = false;
            
            
            
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            double speed=0.4;
            robot.motorFrontLeft.setPower(speed*frontLeftPower);
            robot.motorBackLeft.setPower(speed*backLeftPower);
            robot.motorFrontRight.setPower(speed*frontRightPower);
            robot.motorBackRight.setPower(speed*-1*backRightPower);
            
            if(gamepad1.y){
                cycle = false;
            }
            
            if(gamepad1.b){
                speed=0.9;
            }
            else if(gamepad1.x){
                speed=0.2;
            }
            else{
                speed= 0.6;
            }
            
            
            if(gamepad1.left_bumper){
                robot.viper.setPower(1.0);
                lastDir = true;
               
            }
            
            else if(gamepad1.right_bumper){
                robot.viper.setPower(-1.0);
                lastDir = false;
                //sleep(50);
            }
            else if(lastDir){
                robot.viper.setPower(0.05);
            }
            else{
                robot.viper.setPower(0.0);
            }
            sleep(100);
            
            if(gamepad1.right_trigger >= 0.1){
                robot.claw.setPosition(0.8);
                closed = true;
            }
            else if(gamepad1.left_trigger >= 0.1){
                robot.claw.setPosition(0.2);
                closed = false;
                telemetry.addData("closed", gamepad1.left_trigger);
                telemetry.update();
            }
            
            
            
          
           
            if(gamepad1.right_bumper && gamepad1.left_bumper){
                break;
            }
        }
        
        reset();
    }
    
    public static double absoluteval(double i){
        if(i<0){
            return -i;
        }
        return i;
    }
    
    public void reset(){
        /*robot.leftClaw.setPosition(1.0);
        while(robot.touch.getState()==true){
            robot.leftArm.setPower(0.4);
        }
        robot.leftArm.setPower(0.0);*/
        
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