package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.util.InterpLUT;

public final class distanceLUT {

    // ================= RPM LUT =================
    private static final InterpLUT rpmLUT = new InterpLUT();

    static {
        rpmLUT.add(0,     2000);   // clamp low
        rpmLUT.add(46.2,  2000);
        rpmLUT.add(55.2,  2100);
        rpmLUT.add(67.1,  2200);
        rpmLUT.add(75.1,  2300);
        rpmLUT.add(85.4,  2400);
        rpmLUT.add(94.8,  2500);
        rpmLUT.add(105.9, 2600);
        rpmLUT.add(114.4, 2700);
        rpmLUT.add(125.5, 2825);
        rpmLUT.add(130.3, 2950);
        rpmLUT.add(136.1, 3000);
        rpmLUT.add(140.8, 3050);
        rpmLUT.add(145.0, 3100);
        rpmLUT.add(200,   3100);   // clamp high
        rpmLUT.createLUT();
    }

    private distanceLUT() {}

    public static double getRPM(double distance) {
        return rpmLUT.get(distance);
    }
}
