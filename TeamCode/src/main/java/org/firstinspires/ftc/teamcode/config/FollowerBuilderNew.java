//package org.firstinspires.ftc.teamcode.config;
//import com.pedropathing.Drivetrain;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.follower.FollowerConstants;
//import com.pedropathing.ftc.drivetrains.Mecanum;
//import com.pedropathing.ftc.drivetrains.MecanumConstants;
//import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
//import com.pedropathing.ftc.localization.constants.OTOSConstants;
//import com.pedropathing.ftc.localization.constants.PinpointConstants;
//import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
//import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
//import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
//import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;
//import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
//import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
//import com.pedropathing.ftc.localization.localizers.ThreeWheelIMULocalizer;
//import com.pedropathing.ftc.localization.localizers.ThreeWheelLocalizer;
//import com.pedropathing.ftc.localization.localizers.TwoWheelLocalizer;
//import com.pedropathing.localization.Localizer;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
///** This is the FollowerBuilderNew.
// * It is used to create Followers with a specific drivetrain + localizer without having to use a full constructor
// *
// * @author Baron Henderson - 20077 The Indubitables
// */
//public class FollowerBuilderNew {
//    private final FollowerConstants constants;
//    private PathConstraints constraints;
//    private final HardwareMap hardwareMap;
//    private Localizer localizer;
//    private Drivetrain drivetrain;
//
//    public FollowerBuilderNew(FollowerConstants constants, HardwareMap hardwareMap) {
//        this.constants = constants;
//        this.hardwareMap = hardwareMap;
//        constraints = PathConstraints.defaultConstraints;
//    }
//
//    public FollowerBuilderNew setLocalizer(Localizer localizer) {
//        this.localizer = localizer;
//        return this;
//    }
//
//    public FollowerBuilderNew driveEncoderLocalizer(DriveEncoderConstants lConstants) {
//        return setLocalizer(new DriveEncoderLocalizer(hardwareMap, lConstants));
//    }
//
//    public FollowerBuilderNew OTOSLocalizer(OTOSConstants lConstants) {
//        return setLocalizer(new OTOSLocalizer(hardwareMap, lConstants));
//    }
//
//    public FollowerBuilderNew pinpointLocalizer(PinpointConstants lConstants) {
//        return setLocalizer(new PinpointLocalizerNew(hardwareMap, lConstants));
//    }
//
//    public FollowerBuilderNew threeWheelIMULocalizer(ThreeWheelIMUConstants lConstants) {
//        return setLocalizer(new ThreeWheelIMULocalizer(hardwareMap, lConstants));
//    }
//
//    public FollowerBuilderNew threeWheelLocalizer(ThreeWheelConstants lConstants) {
//        return setLocalizer(new ThreeWheelLocalizer(hardwareMap, lConstants));
//    }
//
//    public FollowerBuilderNew twoWheelLocalizer(TwoWheelConstants lConstants) {
//        return setLocalizer(new TwoWheelLocalizer(hardwareMap, lConstants));
//    }
//
//    public FollowerBuilderNew setDrivetrain(Drivetrain drivetrain) {
//        this.drivetrain = drivetrain;
//        return this;
//    }
//
//    public FollowerBuilderNew mecanumDrivetrain(MecanumConstants mecanumConstants) {
//        return setDrivetrain(new Mecanum(hardwareMap, mecanumConstants));
//    }
//
//    public FollowerBuilderNew pathConstraints(PathConstraints pathConstraints) {
//        this.constraints = pathConstraints;
//        PathConstraints.setDefaultConstraints(pathConstraints);
//        return this;
//    }
//
//    public Follower build() {
//        return new Follower(constants, localizer, drivetrain, constraints);
//    }
//}
