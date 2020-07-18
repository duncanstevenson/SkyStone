package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;

import java.sql.RowId;
import java.util.HashMap;

public class DriveSubsystem {
    private DcMotor leftDriveF = null;
    private DcMotor leftDriveB = null;
    private DcMotor rightDriveF = null;
    private DcMotor rightDriveB = null;
    private boolean isAuto;

    public DriveSubsystem(boolean autonomous){
        isAuto = autonomous;
    }

    public void Periodic() {

    }

    public void initMotors(HardwareMap hardwareMap) {
        leftDriveF = hardwareMap.get(DcMotor.class, "LDF");
        leftDriveB = hardwareMap.get(DcMotor.class, "LDB");
        rightDriveF = hardwareMap.get(DcMotor.class, "RDF");
        rightDriveB = hardwareMap.get(DcMotor.class, "RDB");

        leftDriveF.setDirection(DcMotor.Direction.REVERSE);
        leftDriveB.setDirection(DcMotor.Direction.REVERSE);
        rightDriveF.setDirection(DcMotor.Direction.FORWARD);
        rightDriveB.setDirection(DcMotor.Direction.FORWARD);

        if (isAuto) {
            leftDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void OPDrive(double Left, double Right) {
        leftDriveF.setPower(Left);
        leftDriveB.setPower(Left);
        rightDriveF.setPower(Right);
        rightDriveB.setPower(Right);
    }

    public void POSDrive(double Speed, int LeftPos, int RightPos){
        leftDriveF.setTargetPosition(LeftPos);
        leftDriveF.setPower(Math.abs(Speed));
        leftDriveB.setTargetPosition(LeftPos);
        leftDriveB.setPower(Math.abs(Speed));
        rightDriveF.setTargetPosition(RightPos);
        rightDriveF.setPower(Math.abs(Speed));
        rightDriveB.setTargetPosition(RightPos);
        rightDriveB.setPower(Math.abs(Speed));

        leftDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean atTarget(){
        return leftDriveF.isBusy() && leftDriveB.isBusy() && rightDriveF.isBusy() && rightDriveB.isBusy();
    }
    public int leftFPos(){
        return leftDriveF.getCurrentPosition();
    }
    public int leftBPos(){
        return leftDriveB.getCurrentPosition();
    }
    public int rightFPos(){
        return rightDriveF.getCurrentPosition();
    }
    public int rightBPos(){
        return rightDriveB.getCurrentPosition();
    }

    public void ResetEnc(){
        leftDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
