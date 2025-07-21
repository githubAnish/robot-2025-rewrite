package org.frogforce503.robot2025.offsets;

import java.nio.file.Paths;

import org.frogforce503.robot2025.commands.coral_score_reef.Branch;
import org.frogforce503.robot2025.commands.coral_score_reef.ReefSide;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Filesystem;

public class OffsetsIOServer implements OffsetsIO {
    private final String toRobotTable = "/ReefControls/ToRobot";
    private final String toDashboardTable = "/ReefControls/ToRobot";
    private final String tuningTopicName = "Tuning";
    private final String selectedBranchTopicName = "Branch";
    private final String directionTopicName = "Direction";
    private final String valueTopicName = "Value";

    private final BooleanSubscriber tuningIn;
    private final StringSubscriber selectedBranchIn;
    private final StringSubscriber directionIn;
    private final DoubleSubscriber valueIn;

    private final BooleanPublisher tuningOut;
    private final StringPublisher selectedBranchOut;
    private final StringPublisher directionOut;
    private final DoublePublisher valueOut;

    public OffsetsIOServer() {
        // Create subscribers
        var inputTable = NetworkTableInstance.getDefault().getTable(toRobotTable);
        tuningIn =
            inputTable
                .getBooleanTopic(tuningTopicName)
                .subscribe(false);
        selectedBranchIn =
            inputTable
                .getStringTopic(selectedBranchTopicName)
                .subscribe("", PubSubOption.keepDuplicates(true));
        directionIn =
            inputTable
                .getStringTopic(directionTopicName)
                .subscribe("LEFT");
        valueIn =
            inputTable
                .getDoubleTopic(valueTopicName)
                .subscribe(0.0);

        // Create publishers
        var outputTable = NetworkTableInstance.getDefault().getTable(toDashboardTable);
        tuningOut =
            outputTable
                .getBooleanTopic(tuningTopicName)
                .publish();
        selectedBranchOut =
            outputTable
                .getStringTopic(selectedBranchTopicName)
                .publish();
        directionOut =
            outputTable
                .getStringTopic(directionTopicName)
                .publish();
        valueOut =
            outputTable
                .getDoubleTopic(valueTopicName)
                .publish();

        // Set initial values
        tuningOut.set(false);
        selectedBranchOut.set(""); // Default value for branch
        directionOut.set("LEFT");
        valueOut.set(0.0);

        // Start web server
        WebServer.start(
            5801,
            Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "reefcontrols")
                .toString());
    }

    @Override
    public void updateInputs(OffsetsIOInputs inputs) {
        inputs.data =
            new OffsetsIOData(
                // tuningIn.readQueue().length > 0
                    tuningIn.get(),
                    // : false,
                // selectedBranchIn.readQueue().length > 0
                    selectedBranchIn.get(),
                    // : "",
                // directionIn.readQueue().length > 0
                    directionIn.get(),
                    // : "",
                // valueIn.readQueue().length > 0
                    valueIn.get());
                    // : 0.0);
    }

    @Override
    public void setTuning(boolean set) {
        tuningOut.set(set);
    }

    @Override
    public void setSelectedBranchId(ReefSide side, Branch branch) {
        selectedBranchOut.set(branch.name() + "_" + side.name());
    }

    @Override
    public void setDirection(Direction direction) {
        directionOut.set(direction.name());
    }

    @Override
    public void setValue(double value) {
        valueOut.set(value);
    }
}