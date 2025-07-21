// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import { NT4_Client } from "./NT4.js";

// ***** NETWORKTABLES *****

const toRobotPrefix = "/ReefControls/ToRobot/";
const toDashboardPrefix = "/ReefControls/ToDashboard/";
const tuningTopicName = "Tuning";
const selectedBranchTopicName = "Branch";
const directionTopicName = "Direction";
const offsetValueTopicName = "Value";
const allianceColorTopicName = "/AdvantageKit/RealOutputs/Alliance Color";

const ntClient = new NT4_Client(
  window.location.hostname,
  "ReefControls",
  () => {
    // Topic announce
  },
  () => {
    // Topic unannounce
  },
  (topic, _, value) => {
    // New data
    if (topic.name === toDashboardPrefix + tuningTopicName) {
      tuning = value;
    } else if (topic.name === toDashboardPrefix + selectedBranchTopicName) {
      selectedBranch = value;
    } else if (topic.name === toDashboardPrefix + directionTopicName) {
      direction = value;
    } else if (topic.name === toDashboardPrefix + offsetValueTopicName) {
      offsetValue = value;
    } else if (topic.name === allianceColorTopicName) {
      const allianceBox = document.querySelector(
        ".overview-section div.left-container:nth-child(4) div.left-section"
      );
      allianceBox.style.setProperty("--alliance-color", value === "Red" ? "red" : "rgb(23, 23, 250)");
    } else {
      return;
    }
    updateUI();
  },
  () => {
    // Connected
    document.body.style.backgroundColor = "grey";
  },
  () => {
    // Disconnected
    document.body.style.backgroundColor = "red";
  }
);

// Start NT connection
window.addEventListener("load", () => {
  ntClient.subscribe(
    [
      toDashboardPrefix + tuningTopicName,
      toDashboardPrefix + selectedBranchTopicName,
      toDashboardPrefix + directionTopicName,
      toDashboardPrefix + offsetValueTopicName,
      allianceColorTopicName,
    ],
    false,
    false,
    0.02
  );

  ntClient.publishTopic(toRobotPrefix + tuningTopicName, "boolean");
  ntClient.publishTopic(toRobotPrefix + selectedBranchTopicName, "string");
  ntClient.publishTopic(toRobotPrefix + directionTopicName, "string");
  ntClient.publishTopic(toRobotPrefix + offsetValueTopicName, "double");
  ntClient.connect();
});

// ***** STATE CACHE *****

let tuning = false;
let selectedBranch = "";
let direction = "";
let offsetValue = 0;

/** Update the full UI based on the state cache. */
function updateUI() {
  // Update coral buttons
  
}

// ***** BUTTON BINDINGS *****

let isTouch = false;

function bind(element, callback) {
  let activate = (touchEvent) => {
    if (touchEvent) {
      isTouch = true;
    }
    if (isTouch == touchEvent) {
      callback();
    }
  };

  element.addEventListener("touchstart", () => activate(true));
  element.addEventListener("mousedown", () => activate(false));
  element.addEventListener("contextmenu", (event) => {
    event.preventDefault();
    activate(false);
  });
}

let lastMouseEvent = 0;
window.addEventListener("mousemove", () => {
  let now = new Date().getTime();
  if (now - lastMouseEvent < 50) {
    isTouch = false;
  }
  lastMouseEvent = now;
});

window.addEventListener("load", () => {
  // Add toggle functionality for the most bottom box on the left
  let toggleState = false; // Initial state for the toggle
  const bottomLeftBox = document.querySelector(
    ".overview-section div.left-container:nth-child(1)"
  );
  bottomLeftBox.addEventListener("click", () => {
    toggleState = !toggleState; // Toggle the state
    bottomLeftBox.classList.toggle("active", toggleState); // Update visual state
    console.log("Toggle State:", toggleState); // Log the state for debugging
  });

  // Update the color of the most bottom box on the left based on tuning state
  const updateBottomLeftBoxColor = () => {
    bottomLeftBox.style.setProperty("--box-color", tuning ? "rgb(0, 255, 0)" : "red");
  };

  // Initial color update
  updateBottomLeftBoxColor();

  // Add click event to toggle tuning state and update color
  bottomLeftBox.addEventListener("click", () => {
    tuning = !tuning; // Toggle the tuning state
    updateBottomLeftBoxColor(); // Update the color
    ntClient.addSample(toRobotPrefix + tuningTopicName, tuning); // Publish the updated tuning state
  });

  // Update direction state and publish to NetworkTables
  const directionSelect = document.querySelector(".lower-section .direction select");
  directionSelect.addEventListener("change", () => {
    direction = directionSelect.value; // Update direction state
    ntClient.addSample(toRobotPrefix + directionTopicName, direction); // Publish updated direction
  });

  // Update offset value and publish to NetworkTables
  const offsetInput = document.querySelector(".lower-section .value input[type='text']");
  const applyOffsetButton = document.querySelector(".lower-section .submit button");
  applyOffsetButton.addEventListener("click", () => {
    const offsetValueInput = parseFloat(offsetInput.value);
    if (!isNaN(offsetValueInput)) {
      offsetValue = offsetValueInput; // Update offset value state
      ntClient.addSample(toRobotPrefix + offsetValueTopicName, offsetValue); // Publish updated offset value
    } else {
      console.error("Invalid offset value:", offsetInput.value); // Log error for invalid input
    }
  });

  // Function to convert branch ID to include alliance color
  function getBranchWithAllianceColor(branchId) {
    const allianceColor = document
      .querySelector(".overview-section div.left-container:nth-child(4) div.left-section")
      .style.getPropertyValue("--alliance-color") === "red"
      ? "RED"
      : "BLUE";
    const split = branchId.split("_");
    return `${split[0]}_${allianceColor}_${split[1]}`;
  }

  // Update branch state and publish to NetworkTables
  Array.from(document.getElementsByClassName("branch")).forEach((element) => {
    bind(element, () => {
      selectedBranch = getBranchWithAllianceColor(element.id); // Update selected branch state
      ntClient.addSample(toRobotPrefix + selectedBranchTopicName, selectedBranch); // Publish updated branch
    });
  });

  // Update coral buttons to toggle coral image
  Array.from(document.getElementsByClassName("branch")).forEach((element) => {
    bind(element, () => {
      // Remove coral image from all buttons
      Array.from(document.getElementsByClassName("branch")).forEach((btn) => {
        btn.classList.remove("active");
      });

      // Add coral image to the currently pressed button
      element.classList.add("active");
    });
  });
});

// Function to update the selected branch's color based on alliance color
function updateSelectedBranchColor() {
  const allianceColor = document
    .querySelector(".overview-section div.left-container:nth-child(4) div.left-section")
    .style.getPropertyValue("--alliance-color");

  const selectedBranchElement = document.querySelector(".branch.active");
  if (selectedBranchElement) {
    selectedBranchElement.style.backgroundColor = allianceColor;
  }
}

// Observe changes to the alliance color and update the selected branch color
window.addEventListener("load", () => {
  const allianceBox = document.querySelector(
    ".overview-section div.left-container:nth-child(4) div.left-section"
  );

  const observer = new MutationObserver(() => {
    updateSelectedBranchColor(); // Update branch color when alliance color changes
  });

  observer.observe(allianceBox, { attributes: true, attributeFilter: ["style"] });
});

// ***** REEF CANVAS *****

window.addEventListener("load", () => {
  const canvas = document.getElementsByTagName("canvas")[0];
  const context = canvas.getContext("2d");

  let render = () => {
    const devicePixelRatio = window.devicePixelRatio;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    canvas.width = width * devicePixelRatio;
    canvas.height = height * devicePixelRatio;
    context.scale(devicePixelRatio, devicePixelRatio);
    context.clearRect(0, 0, width, height);

    const corners = [
      [width * 0.74, height * 0.9],
      [width * 0.26, height * 0.9],
      [width * 0.03, height * 0.5],
      [width * 0.26, height * 0.1],
      [width * 0.74, height * 0.1],
      [width * 0.97, height * 0.5],
    ];

    context.beginPath();
    corners.forEach((corner) => {
      context.moveTo(width * 0.5, height * 0.5);
      context.lineTo(...corner);
    });
    corners.forEach((corner, index) => {
      if (index == 0) {
        context.moveTo(...corner);
      } else {
        context.lineTo(...corner);
      }
    });
    context.closePath();

    context.strokeStyle = "black";
    context.stroke();

    // Draw branch IDs
    const reefSectionRect = document.querySelector(".reef-section").getBoundingClientRect();
    const canvasRect = canvas.getBoundingClientRect();
    const branches = Array.from(document.getElementsByClassName("branch"));
    branches.forEach((branch) => {
      const branchRect = branch.getBoundingClientRect();
      const x = branchRect.left + branchRect.width / 2 - canvasRect.left;
      const y = branchRect.top - canvasRect.top - 5; // Position text slightly above the branch

      context.font = "10px Arial"; // Small font size
      context.fillStyle = "black";
      context.textAlign = "center";
      context.fillText(branch.id, x, y);
    });
  };

  render();
  window.addEventListener("resize", render);
});
