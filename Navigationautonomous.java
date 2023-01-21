/*
Copyright (c) 2018 FIRST
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.util.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link SensorREV2mDistance} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 */
@Autonomous(name = "NavigationAutonomous", group = "Sensor")

public class Navigationautonomous extends LinearOpMode {
    HardwarePushbot robot   = new HardwarePushbot(); 
    MeccanumTest mc= new MeccanumTest();
    
    public  int numTiles = 6;
    // public static int[][] tiles = {
    //         {0, 0, 0, 0, 0, 0},
    //         {0, 0, 0, 0, 0, 0},
    //         {0, 0, 0, 0, 0, 0},
    //         {0, 0, 0, 0, 0, 0},
    //         {0, 0, 0, 0, 0, 0},
    //         {0, 0, 0, 0, 0, 0}
    //         }; // 1 is obstacle
    public static int[][]tiles=new int[6][6];
    @Override
    public void runOpMode() {
        
        // you can use this as a regular DistanceSensor.
        // sensorRange = hardwareMap.get(DistanceSensor.class, "Infared");

        // // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // // methods associated with the Rev2mDistanceSensor class.
        // Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        // telemetry.addData(">>", "Press start to continue");
        // telemetry.update();

        // waitForStart();
        // while(opModeIsActive()) {
        //     // generic DistanceSensor methods.
        //     telemetry.addData("deviceName",sensorRange.getDevi   ceName() );
        //     telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
        //     telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        //     telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
        //     telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

        //     // Rev2mDistanceSensor specific methods.
        //     telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        //     telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

        //     telemetry.update();
        // }
        waitForStart();
        robot.init(hardwareMap);
        driveToTile(0  , 0, 4, 1);
        
        sleep(1000);
        
        
    }
    public String driveToTile(int xBegin, int yBegin, int xEnd, int yEnd) {
   
        int curI = xBegin;
        int curJ = yBegin;
        String finalMoves = ""; 
        
        while(curI != xEnd || curJ != yEnd) {
            senseMap(curI, curJ); // update map obstacles with sensors
            
            String path = bfs(curI, curJ, xEnd, yEnd); // recalculate path
            try{
                if(path.length()==0){
                    break;
               
                }
            }
            catch(Exception e){
                break;
            }
            telemetry.addData("Path:",path);
            telemetry.update();
            sleep(100);
            if(path.charAt(0) == 'R') {
                curJ += 1;
                SlideRight(0.3);
                robotsleep(1440);
                CancelPowerRobot();
                
            }
            if(path.charAt(0) == 'D') {
                curI += 1;
                RearDrive(0.3);
                robotsleep(1440);
                CancelPowerRobot();
            }
            if(path.charAt(0) == 'L') {
                curJ -= 1;
                SlideLeft(0.3);
                robotsleep(1440);
                CancelPowerRobot();
            }
            if(path.charAt(0) == 'U') {
                curI -= 1; // change spot based on path
                FrontDrive(0.3);
                robotsleep(1440);
                CancelPowerRobot();
            }
            finalMoves += path.charAt(0); // add to final moves
            
        }
        
        return finalMoves;
        
    }
    public void senseMap(int x, int y) {
        
        // enter taruns code -> will sense 2 spots in all 4 directions, update tiles with a 1 if there is an obstacle and a 0 if no
        double Right=robot.iR.getDistance(DistanceUnit.INCH);
        double Back=robot.iB.getDistance(DistanceUnit.INCH);
        double Left=robot.iL.getDistance(DistanceUnit.INCH);
        double Front=robot.iF.getDistance(DistanceUnit.INCH);

        // each tile is 2 ft
        
        int rslide=Math.max(1,(int)(Math.ceil(Right/24)));
        int bslide=Math.max(1,(int)(Math.ceil(Back/24)));
        int lslide=Math.max(1,(int)(Math.ceil(Left/24)));
        int fslide=Math.max(1,(int)(Math.ceil(Front/24)));
       
        // sleep(4000);
        if((rslide == 1 || rslide == 2) && (y+rslide<6)) {
            tiles[x][y + rslide]=1;
            
        }
        if((bslide == 1 || bslide == 2) && (x+bslide<=0)){
            tiles[x+bslide][y]=1;
        }
        if((lslide == 1 || lslide == 2) && (y-lslide>=0)){
            tiles[x][y-lslide]=1;
        }
        if((fslide == 1 || fslide == 2) && (x-fslide>=0)){
            
            tiles[x-fslide][y]=1;
        }
        
        
       
        
        
            for(int i = 0; i < 6; i++){
                telemetry.addData(("line" + String.valueOf(i)), Arrays.toString(tiles[i]));
            }
            telemetry.addData("rslide:", rslide);
            telemetry.addData("bslide:", bslide);
            telemetry.addData("lslide:", lslide);
            telemetry.addData("Fslide: ", fslide);
            telemetry.addData("X",x);
            
            telemetry.update();
        sleep(10000);
        
        
    }
    public String bfs(int xStart, int yStart, int xEnd, int yEnd) {
        Queue<tile> q = new ArrayDeque<tile>(); // Queue (first in first out) for BFS
        String[][] paths = new String[numTiles][numTiles]; // Paths 2D array to keep track of paths to each tile
        int[][] seen = new int[numTiles][numTiles]; // seen -> separate from tiles
        for(int i = 0; i < numTiles; i++) for(int j = 0; j < numTiles; j++) seen[i][j] = tiles[i][j];
        q.add(new tile(xStart, yStart, "")); // add starting tile to queue
        seen[xStart][yStart] = 1; // set starting tile to seen
        while(!q.isEmpty()) { // keep going until queue is exhausted
            tile cur = q.poll(); // get front element of queue
            for(int i = -1; i <= 1; i++) {
                for(int j = -1; j <= 1; j++) {
                    if(Math.abs(i) != Math.abs(j) && 0 <= cur.x+i && cur.x+i <= numTiles-1 && 0 <= cur.y+j && cur.y+j <= numTiles-1 && seen[cur.x+i][cur.y+j] == 0) {
                        // in all 4 directions, check if in bounds & not seen
                        String newMove = cur.moves;
                        if(i == 0 && j == 1) newMove += "R";
                        if(i == 1 && j == 0) newMove += "D";
                        if(i == 0 & j == -1) newMove += "L";
                        if(i == -1 && j == 0) newMove += "U"; // add this move
                        q.add(new tile(cur.x + i, cur.y + j, newMove)); // add to queue
                        seen[cur.x + i][cur.y + j] = 1; // set seen
                        paths[cur.x + i][cur.y + j] = newMove; // update paths
                    }
                }
            }

        }
        return paths[xEnd][yEnd]; // return path to the target spot
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


class tile{
    public int x;
    public int y;
    public String moves;
    boolean visited = false;
    tile(int x, int y, String moves){ // constructor -> sets x, y, and moves for each tile
        this.x = x;
        this.y = y;
        this.moves = moves;
    }
}