// Made by Mohnish Sivakumar (FTC Programming Head, Co-Secretary)
package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import fi.iki.elonen.NanoHTTPD;

/**
 * Made by Mohnish Sivakumar (Co-Sofware Head)
 * LiveAuto2 OpMode allows live configuration of autonomous trajectories via a web interface.
 * It uses NanoHTTPD to serve an HTML page where users can input up to 6 points (X, Y, Heading).
 * The robot builds and follows the trajectory based on the filled points when "Run" is pressed.
 */
@TeleOp(name = "LiveAuto", group = "Tests")
public class LiveAuto extends OpMode {
    private NanoHTTPD nanoServer;
    private String[][] points = new String[6][3]; // Array to store 6 points (X, Y, Heading)
    private List<Pose2d> filteredPoints = new ArrayList<>(); // List to store only filled points

    private MecanumDrive drive;
    private boolean runButtonPressed = false;
    private boolean actionRunning = false;
    private Action trajectoryAction;

    @Override
    public void init() {

        // Initialize the Roadrunner drive at a particular pose
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize points array with empty strings to prevent nulls
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                points[i][j] = "";
            }
        }

        try {
            // Start the NanoHTTPD server on the specified IP and port
            nanoServer = new NanoHTTPD("192.168.43.1", 8043) {
                @Override
                public Response serve(IHTTPSession session) {
                    Map<String, String> parms = session.getParms();

                    // Update points array with received parameters
                    if (!parms.isEmpty()) {
                        for (int i = 0; i < 6; i++) {
                            points[i][0] = parms.getOrDefault("x-pos-" + i, "").trim();
                            points[i][1] = parms.getOrDefault("y-pos-" + i, "").trim();
                            points[i][2] = parms.getOrDefault("heading-" + i, "").trim();
                        }

                        // If "Run" button was pressed, set the flag
                        runButtonPressed = "true".equals(parms.get("push-button"));
                    }

                    // Build the HTML response with the updated trajectory action
                    StringBuilder msg = new StringBuilder();
                    msg.append("<html><head><title>LiveAuto</title>")
                            .append("<style>") // Existing styles
                            .append("body {margin:0;padding:0;font-family:'Trebuchet MS',sans-serif;display:flex;}")
                            .append(".point-box {background-color:#fff;border-radius:30px;padding:20px;border:1px solid #ddd;box-shadow:0 0 10px rgba(0,0,0,0.2);font-size:12px;margin:10px;height:240px;width:300px;}")
                            .append(".input-field {width:80px;height:30px;padding:5px;border:1px solid #ccc;border-radius:5px;font-size:14px;}")
                            .append(".input-row {display:flex;flex-direction:column;align-items:flex-start;margin-bottom:10px;}")
                            .append("#push-button {background-color:#4CAF50;color:#fff;padding:10px 20px;font-size:15px;border:none;border-radius:20px;width:180px;height:150px;cursor:pointer;margin:10px;position:absolute;top:95%}")
                            .append("#update-button {background-color:#FF5733;color:#fff;padding:10px 20px;font-size:15px;border:none;border-radius:20px;width:180px;height:150px;cursor:pointer;margin:10px;position:absolute;top:95%;left:200px;}") // Updated button styles
                            .append(".point-row {display:grid;grid-template-columns: repeat(3, 1fr);grid-gap: 10px;margin-bottom:20px;}")
                            .append("#trajectory-action {background-color:#fff;border-radius:30px;height:500px;padding:15px;border:1px solid #ddd;box-shadow:0 0 10px rgba(0,0,0,0.2);width:630px;font-size:12px;margin:20px;position:absolute;right:20%;top:85%;height:200px}")
                            .append("#trajectory-action-text {width:100%;height:100%;resize:none;}")
                            .append("#point-form {display:flex;flex-direction:column;justify-content:space-between;width:50%;}")
                            .append("</style></head><body>")
                            .append("<form id=\"point-form\">")
                            .append("<div class=\"point-row\">")
                            .append("<div class=\"point-box\"><h1>Starting Point</h1><div class=\"input-row\">")
                            .append("<label>X position:</label><input type=\"number\" step=\"0.1\" class=\"input-field\" id=\"x-pos-0\" name=\"x-pos-0\" value=\"")
                            .append(points[0][0]).append("\">")
                            .append("<label>Y position:</label><input type=\"number\" step=\"0.1\" class=\"input-field\" id=\"y-pos-0\" name=\"y-pos-0\" value=\"")
                            .append(points[0][1]).append("\">")
                            .append("<label>Heading:</label><input type=\"number\" step=\"0.1\" class=\"input-field\" id=\"heading-0\" name=\"heading-0\" value=\"")
                            .append(points[0][2]).append("\"></div></div>");

                    for (int i = 1; i < 6; i++) {
                        msg.append("<div class=\"point-box\"><h1>Point ").append(i).append("</h1><div class=\"input-row\">")
                                .append("<label>X position:</label><input type=\"number\" step=\"0.1\" class=\"input-field\" id=\"x-pos-").append(i)
                                .append("\" name=\"x-pos-").append(i).append("\" value=\"").append(points[i][0]).append("\">")
                                .append("<label>Y position:</label><input type=\"number\" step=\"0.1\" class=\"input-field\" id=\"y-pos-")
                                .append(i).append("\" name=\"y-pos-").append(i).append("\" value=\"").append(points[i][1]).append("\">")
                                .append("<label>Heading:</label><input type=\"number\" step=\"0.1\" class=\"input-field\" id=\"heading-")
                                .append(i).append("\" name=\"heading-").append(i).append("\" value=\"").append(points[i][2]).append("\"></div></div>");
                    }

                    msg.append("</div>")
                            .append("<div id=\"trajectory-action\" style=\"margin-left:50%;\"><h1>Trajectory Action:</h1><textarea id=\"trajectory-action-text\" readonly>");

                    // Initial trajectory action placeholder
                    msg.append("// Fill in all fields to build a trajectory");

                    msg.append("</textarea></div>")
                            .append("<button type=\"submit\" id=\"push-button\" name=\"push-button\" value=\"true\">Run</button>")
                            .append("<button type=\"button\" id=\"update-button\" onclick=\"updateAction();\">Update Action</button>")
                            .append("<script>")
                            .append("function updateAction() {") // JavaScript for updating the trajectory
                            .append("var trajectoryText = '';") // Variable to hold trajectory action text
                            .append("var points = [];");

                    for (int i = 0; i < 6; i++) {
                        msg.append("var x = document.getElementById('x-pos-").append(i).append("').value.trim();")
                                .append("var y = document.getElementById('y-pos-").append(i).append("').value.trim();")
                                .append("var heading = document.getElementById('heading-").append(i).append("').value.trim();")

                                // Only add the point if x, y, and heading are filled
                                .append("if (x !== '' && y !== '' && heading !== '') {")
                                .append("points.push({x: parseFloat(x), y: parseFloat(y), heading: parseFloat(heading)});")
                                .append("}");
                    }

                    msg.append("if (points.length >= 2) {") // Ensure at least two valid points
                            .append("trajectoryText += 'drive.actionBuilder(new Pose2d(' + points[0].x + ', ' + points[0].y + ', ' + points[0].heading + '))\\n';")
                            .append("for (var i = 1; i < points.length; i++) {")
                            .append("trajectoryText += '.strafeToLinearHeading(new Vector2d(' + points[i].x + ', ' + points[i].y + '), Math.toRadians(' + points[i].heading + '))\\n';")
                            .append("}")
                            .append("trajectoryText += '.build();';")
                            .append("document.getElementById('trajectory-action-text').value = trajectoryText;") // Update the textarea with the trajectory action
                            .append("alert('Trajectory action updated!');") // Confirmation alert
                            .append("} else {")
                            .append("alert('Please fill in at least two points to update the trajectory action.');")
                            .append("}")
                            .append("}")
                            .append("</script>")
                            .append("</form></body></html>");

                    // Respond with the generated HTML
                    return newFixedLengthResponse(Response.Status.OK, "text/html", msg.toString());
                }
            };

            nanoServer.start();
            telemetry.addData("NanoHTTPD", "Server started on 192.168.43.1:8043");
            telemetry.update();
        } catch (IOException ioe) {
            telemetry.addData("Error", "Failed to start server: " + ioe.getMessage());
            telemetry.update();
            ioe.printStackTrace();
        }
    }

    @Override
    public void loop() {
        // Only proceed if "Run" has been pressed and no trajectory is currently running
        if (runButtonPressed && !actionRunning) {
            try {
                // Clear the filteredPoints list to update with new points
                filteredPoints.clear();

                // Create a filtered list excluding any empty points
                for (int i = 0; i < 6; i++) {
                    if (isPointFilled(i)) {
                        try {
                            double x = Double.parseDouble(points[i][0]);
                            double y = Double.parseDouble(points[i][1]);
                            double heading = Math.toRadians(Double.parseDouble(points[i][2]));
                            filteredPoints.add(new Pose2d(x, y, heading));
                        } catch (NumberFormatException e) {
                            telemetry.addData("Error", "Invalid number format at Point " + (i + 1));
                        }
                    }
                }

//                // Ensure there are at least two valid points to build a trajectory
                if (filteredPoints.size() < 2) {
                    telemetry.addData("Error", "At least two points required to run trajectory");
                } else {
                    // Set the robot's starting pose
                    Pose2d startPose = filteredPoints.get(0);
                    drive.pose = startPose;

                    // Initialize the TrajectoryActionBuilder with the starting pose
                    TrajectoryActionBuilder builder = drive.actionBuilder(startPose);

                    // Add each subsequent point to the trajectory

                    for (int i = 1; i < filteredPoints.size(); i++) {
                        Pose2d targetPose = filteredPoints.get(i);
                        builder = builder.strafeToLinearHeading(targetPose.position, targetPose.heading);
                    }

//                  // Build the trajectory action
                    trajectoryAction = builder.build();
                    telemetry.addData("Trajectory action", "Ready to run");

                    // Start following the trajectory action
                    actionRunning = true;

                    telemetry.addData("Action", "Trajectory execution started");
                }
//
//                // Reset the run button flag after the action is built and running
                runButtonPressed = false;
            } catch (Exception e) {
                telemetry.addData("Error", "Failed to build or run trajectory action: " + e.getMessage());
                runButtonPressed = false; // Reset the run button flag to allow retrying
            }
        }

        // If a trajectory is currently running, update the drive and check for completion
        if (actionRunning) {

            TelemetryPacket packet = new TelemetryPacket();

            FtcDashboard dash = FtcDashboard.getInstance();

            trajectoryAction.preview(packet.fieldOverlay());

            if (trajectoryAction.run(packet)) {
                telemetry.addData("Trajectory action", "Running...");
            } else {
                telemetry.addData("Trajectory action", "Execution completed");
                actionRunning = false;  // Reset the trajectory running flag
            }

            dash.sendTelemetryPacket(packet);
        }

        // Update telemetry after each loop
        telemetry.update();
    }


    /**
     * Checks if all fields for a given point index are filled.
     *
     * @param index The index of the point to check (0-based).
     * @return true if all fields (X, Y, Heading) are filled; false otherwise.
     */
    private boolean isPointFilled(int index) {
        return points[index][0] != null && !points[index][0].trim().isEmpty() &&
                points[index][1] != null && !points[index][1].trim().isEmpty() &&
                points[index][2] != null && !points[index][2].trim().isEmpty();
    }

    @Override
    public void stop() {
        // Stop the NanoHTTPD server when the OpMode stops
        if (nanoServer != null) {
            nanoServer.stop();
            telemetry.addData("NanoHTTPD", "Server stopped");
            telemetry.update();
        }

        // Reset trajectory-related flags and variables
        trajectoryAction = null;
        runButtonPressed = false;
        actionRunning = false;
    }
}