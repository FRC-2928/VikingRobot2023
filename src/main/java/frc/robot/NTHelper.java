package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// Helper functions for using NT4
public class NTHelper {
    private static NetworkTable poseTable = NetworkTableInstance.getDefault().getTable("Poses");

    public static void sendTagLayout(AprilTagFieldLayout tags) {
        for(AprilTag tag : tags.getTags()) {
            ArrayList<Double> tagPropertyList = new ArrayList<>();

            tagPropertyList.add(tag.pose.getX());
            tagPropertyList.add(tag.pose.getY());
            tagPropertyList.add(tag.pose.getZ());
            tagPropertyList.add(tag.pose.getRotation().getQuaternion().getW());
            tagPropertyList.add(tag.pose.getRotation().getQuaternion().getX());
            tagPropertyList.add(tag.pose.getRotation().getQuaternion().getY());
            tagPropertyList.add(tag.pose.getRotation().getQuaternion().getZ());

            DoubleArrayPublisher tagPub = NTHelper.poseTable.getDoubleArrayTopic("AprilTag" + tag.ID).publish();

            tagPub.getTopic().setRetained(true);
            tagPub.set(tagPropertyList.stream().mapToDouble(Double::doubleValue).toArray());
            tagPub.close();
        }

        IntegerArrayPublisher idPub = NTHelper.poseTable.getIntegerArrayTopic("AprilTag ID").publish();

        idPub.getTopic().setRetained(true);
        idPub.set(new long[] { 1, 2, 3, 5, 6, 7});
        idPub.close();
    }
}
