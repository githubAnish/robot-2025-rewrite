package org.frogforce503.robot2025.subsystems.offsets;

import java.nio.file.Paths;

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
            Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "reefcontrols").toString());
    }

    @Override
    public void updateInputs(OffsetsIOInputs inputs) {
        inputs.tuning = tuningIn.get();
        inputs.branch = selectedBranchIn.get();
        inputs.direction = directionIn.get();
        inputs.value = valueIn.get();
    }

    @Override
    public void setValue(double value) {
        valueOut.set(value);
    }
}