package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double p;
    private double i;
    private double iPersistence = 0;
    private double d;
    private int e;
    private double maxSpeed;
    private int target = 0;
    private int current = 0;
    private double lastError = 0;
    private double ti_error;

    public PID(double P, Double I, double D, int epsion, double MaxSpeed){
        p = P;
        i = I;
        d = D;
        e = epsion;
        maxSpeed = MaxSpeed;
    }

    public void SetTarget(int Target){
        target = Target;
    }

    public double Calculate(int CurrentPos, double elapsedTime){
        current = CurrentPos;
        ti_error = elapsedTime;
        double error = target - current;
        double result = CalcP(error) + CalcI() + CalcD(error);
        if(result > maxSpeed){
            result = maxSpeed;
        }else if(result < -maxSpeed){
            result = -maxSpeed;
        }
        lastError = error;
        return result;
    }

    public double Calculate(int Target, int CurrentPos, double elapsedTime) {
        target = Target;
        return Calculate(CurrentPos, elapsedTime);
    }

    private double CalcP(double Error){
        return Error*p;
    }

    private double CalcI(){
        iPersistence = iPersistence + (i/ti_error)*lastError;
        return iPersistence;
    }

    private double CalcD(double Error){
        double errorError = lastError - Error;
        return (d/ti_error)*errorError;
    }

    public boolean atTarget(){
//        Log.d("PID", "error: "+ (Math.abs(lastError) < e));
        return Math.abs(lastError) < e;
    }

    public void UpdatePIDE(double P, double I, double D, int e){
        p = P;
        i = I;
        d = D;
        this.e = e;
    }

    public void UpdateP(double P){
        p = P;
    }

    public void UpdateI(double I){
        i = I;
    }

    public void UpdateD(double D){
        d = D;
    }

    public void UpdateEpsilon(int Epsilon){
        e = Epsilon;
    }

    public void UpdateMaxSpeed(double max){
        maxSpeed = max;
    }
}
