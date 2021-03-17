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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import java.util.Objects;
import java.util.concurrent.TimeUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="DriveSquare", group="Testing")
//@Disabled
public class DriveSquareAuto extends TunableOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveF = null;
    private DcMotor leftDriveB = null;
    private DcMotor rightDriveF = null;
    private DcMotor rightDriveB = null;
    private int ExecutionPoint = 0;
    private double lastTime;
    private double looptime = 0;
    private HardwareBase robot = new HardwareBase(false);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.drive.initMotors(hardwareMap);
        lastTime = (double) runtime.now(TimeUnit.MILLISECONDS);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        looptime = Math.abs((double) runtime.now(TimeUnit.MILLISECONDS) - lastTime);
        lastTime = (double) runtime.now(TimeUnit.MILLISECONDS);
        telemetry.addData("DriveP", ""+ robot.drive.DriveP);
        robot.drive.DriveP = getDouble("DriveP");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.drive.ResetEnc();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.drive.DriveP = getDouble("DriveP");
        robot.drive.DriveI = getDouble("DriveI");
        robot.drive.DriveD = getDouble("DriveD");
        robot.drive.DriveE = getInt("DriveE");
        robot.drive.UpdateDrivePID();
        looptime = Math.abs((double) runtime.now(TimeUnit.MILLISECONDS) - lastTime);
        lastTime = (double) runtime.now(TimeUnit.MILLISECONDS);

//        if(ExecutionPoint == 0){
//            robot.drive.POSDrive(1, 2000, 2000);
//            ExecutionPoint++;
//        }else if(ExecutionPoint == 1 && robot.drive.atTargetPOS()){
//            robot.drive.ResetEnc();
//            ExecutionPoint++;
//        }else if(ExecutionPoint == 2){
//            robot.drive.POSDrive(1, 1200, 0);
//            ExecutionPoint++;
//        }else if(ExecutionPoint == 3 && robot.drive.atTargetPOS()){
//            robot.drive.ResetEnc();
//            ExecutionPoint = 0;
//        }
        if(ExecutionPoint == 0){
            robot.drive.PIDDrive(1, 3000, 3000, looptime);
            if(robot.drive.atTargetPID()){
                Log.e("DriveSquareAuto", "inc expoint");
                ExecutionPoint++;
            }
        }else if(ExecutionPoint == 1){
            robot.drive.ResetEnc();
            ExecutionPoint++;
        }else if(ExecutionPoint == 2){
            robot.drive.PIDDrive(1, -3000, -3000, looptime);
            if(robot.drive.atTargetPID()){
                ExecutionPoint++;
            }
        }else if(ExecutionPoint == 3){
            robot.drive.ResetEnc();
            ExecutionPoint = 0;
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("PositionFL", "" + robot.drive.leftFPos());
        telemetry.addData("PositionBL", "" + robot.drive.leftBPos());
        telemetry.addData("PositionFR", "" + robot.drive.rightFPos());
        telemetry.addData("PositionBR", "" + robot.drive.rightBPos());
        telemetry.addData("DriveP", ""+ robot.drive.DriveP);
        telemetry.addData("expoint", "" + ExecutionPoint);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.drive.OPDrive(0,0);
    }

}
