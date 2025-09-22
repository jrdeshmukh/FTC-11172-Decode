package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.FirstBoolean;

public class BBG extends Gamepad {
    Gamepad gamepad;

    FirstBoolean dpad_up, dpad_down, dpad_left, dpad_right, a, b, x, y, left_bumper, right_bumper, pss;
    public BBG(Gamepad gamepad) {
        this.gamepad = gamepad;
        dpad_up = new FirstBoolean();
        dpad_down = new FirstBoolean();
        dpad_left = new FirstBoolean();
        dpad_right = new FirstBoolean();
        a = new FirstBoolean();
        b = new FirstBoolean();
        x = new FirstBoolean();
        y = new FirstBoolean();
        left_bumper = new FirstBoolean();
        right_bumper = new FirstBoolean();
        pss = new FirstBoolean();
    }
    public boolean dpad_up() {return dpad_up.betterboolean(gamepad.dpad_up);}
    public boolean dpad_down() {return dpad_down.betterboolean(gamepad.dpad_down);}
    public boolean dpad_left() {return dpad_left.betterboolean(gamepad.dpad_left);}
    public boolean dpad_right() {return dpad_right.betterboolean(gamepad.dpad_right);}
    public boolean a() {return a.betterboolean(gamepad.a);}
    public boolean b() {return b.betterboolean(gamepad.b);}
    public boolean x() {return x.betterboolean(gamepad.x);}
    public boolean y() {return y.betterboolean(gamepad.y);}
    public boolean left_bumper(){return left_bumper.betterboolean(gamepad.left_bumper);}
    public boolean right_bumper() {return right_bumper.betterboolean(gamepad.right_bumper);}
    public boolean ps() {return pss.betterboolean(gamepad.ps);}
}

