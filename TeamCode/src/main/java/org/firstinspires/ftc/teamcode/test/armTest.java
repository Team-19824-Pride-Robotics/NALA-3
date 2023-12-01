package org.firstinspires.ftc.teamcode.test;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(group = "test")
public class armTest extends OpMode {


boolean liftControl = false;


    //arm
    int lift = 0;
    int bumper = 0;

    double aPos =.99;

    public static double bVar = 214;
    public static double bPosx = .6;
    ServoImplEx Arm;
    ServoImplEx bucket;
    // AnalogInput sEncoder;
    AnalogInput sEncoder;
    AnalogInput sEncoder2;






    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//pid

        Arm= (ServoImplEx) hardwareMap.get(Servo.class, "Arm");
        bucket = (ServoImplEx) hardwareMap.get(Servo.class, "bucket");
        Arm.setPwmRange(new PwmControl.PwmRange(505, 2495));
        bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sEncoder = hardwareMap.get(AnalogInput.class, "sEncoder");
        sEncoder2 = hardwareMap.get(AnalogInput.class, "sEncoder2");




    }

    @Override
    public void loop() {






        double pos = sEncoder.getVoltage() / 3.3 * 360;
        double pos2 = sEncoder2.getVoltage() / 3.3 * 360;



        if (gamepad2.right_bumper){
            liftControl = true;
            bumper = 1;
        }
        if (gamepad2.left_bumper){
            liftControl = false;
            bPosx=.6;
            bumper = 2;
        }



        if (liftControl) {

            if (gamepad2.a) {
                aPos = .01;
            }
            if (gamepad2.b){
                lift= 2;
            }

        }

        if (!liftControl) {
             if (bPosx == .6){
                 aPos=.99;
             }
        }


        Arm.setPosition(aPos);


        telemetry.addData("Run time",getRuntime());
        telemetry.addData("armtest1","test");
        telemetry.addData("apos",aPos);
        telemetry.addData("lift",lift);
        telemetry.addData("bumper",bumper);
        telemetry.addData("pos",pos);
        telemetry.addData("pos",pos2);


        telemetry.update();
    }

}