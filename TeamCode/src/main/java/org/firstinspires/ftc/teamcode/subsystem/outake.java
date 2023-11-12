package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
public class outake extends SubsystemBase {

    private final ServoImplEx topbucket;
    private final ServoImplEx buttombucket;

    public outake(final HardwareMap hMap, final String name) {
        topbucket = (ServoImplEx) hMap.get(Servo.class, name);
        buttombucket = (ServoImplEx) hMap.get(Servo.class, name);
        topbucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        buttombucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
    }

    public void tOpen() {
        topbucket.setPosition(0.76);
    }
    public void tClose() {
        topbucket.setPosition(0);
    }
    public void bOpen() {
        buttombucket.setPosition(0.76);
    }
    public void bClose() {
        buttombucket.setPosition(0);
    }

}