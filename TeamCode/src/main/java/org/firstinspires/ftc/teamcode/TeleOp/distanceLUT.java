package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.util.InterpLUT;

public final class distanceLUT {

    // ================= RPM LUT =================
    private static final InterpLUT rpmLUT = new InterpLUT();

    static {
        rpmLUT.add(0,     2000);   // clamp low
        rpmLUT.add(55,  2400);
        rpmLUT.add(64,  2500);
        rpmLUT.add(81,  2500);
        rpmLUT.add(89,  2550);
        rpmLUT.add(99.7,  2600);
        rpmLUT.add(110,  2750);
        rpmLUT.add(122.9, 2900);
        rpmLUT.add(132, 3050);// clamp high
        rpmLUT.createLUT();
    }

    private distanceLUT() {}

    public static double getRPM(double distance) {
        return rpmLUT.get(distance);
    }
}
