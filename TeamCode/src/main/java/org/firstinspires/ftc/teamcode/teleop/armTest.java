package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;


@Config
@TeleOp
public class armTest extends OpMode {

    double aVar;
    public static double bVar = 64;
    public static double bPosx = .18;

    double aPos;

    ServoImplEx lArm;
    ServoImplEx rArm;
    ServoImplEx bucket;
    // AnalogInput sEncoder;
   AnalogInput sEncoder;
   AnalogInput sEncoder2;
    AnalogInput sEncoder3;


    @Override
    public void init() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            lArm= (ServoImplEx) hardwareMap.get(Servo.class, "lArm");
            rArm = (ServoImplEx) hardwareMap.get(Servo.class, "rArm");
            bucket = (ServoImplEx) hardwareMap.get(Servo.class, "bucket");
            lArm.setPwmRange(new PwmControl.PwmRange(505, 2495));
            rArm.setPwmRange(new PwmControl.PwmRange(505, 2495));
            bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
            sEncoder = hardwareMap.get(AnalogInput.class, "sEncoder");
            sEncoder2 = hardwareMap.get(AnalogInput.class, "sEncoder2");
            sEncoder3 = hardwareMap.get(AnalogInput.class, "sEncoder3");


    }

    @Override
    public void loop() {

        double pos = sEncoder.getVoltage() / 3.3 * 360;
        double pos2 = sEncoder2.getVoltage() / 3.3 * 360;
        double pos3 = sEncoder3.getVoltage() / 3.3 * 360;

        aPos =  Range.clip(aPos, .15, .99);
        aVar =  Range.clip(aVar, 53, 355);

        bVar =  Range.clip(bVar, 64, 225);
        bPosx =  Range.clip(bPosx, .15, .6);

        if (gamepad2.right_stick_y < .2 || gamepad2.right_stick_y > -.2) {
            aVar = aVar + gamepad2.right_stick_y *10;
        }
        if (gamepad2.left_stick_y < .2 || gamepad2.left_stick_y > -.2) {
            bVar = bVar + gamepad2.left_stick_y;
        }

        aPos = aVar * .0028;
        bPosx= bVar *.0028;

        lArm.setPosition(aPos);
        rArm.setPosition(aPos);
        bucket.setPosition(bPosx);

        telemetry.addData("Run time",getRuntime());
        telemetry.addData("pos1", pos);
        telemetry.addData("pos2", pos2);
        telemetry.addData("pos3", pos3);
        telemetry.addData("test", 2);
        telemetry.update();
    }


}