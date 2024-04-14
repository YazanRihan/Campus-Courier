const rooms = {
  "M8":
    ["main", "001", "002", "003", "004", "005", "007", "008", "009", "bathroom", "010", "011", "012"],

  "M9":
    ["main", "001", "002", "005", "006", "007", "008", "bathroom", "009", "010", "011"],

  "M10":
    ["main", "001", "002", "003", "004", "005", "006", "007", "008", "bathroom", "009", "010", "011"]
}

// function init() {
//     // Connect to ROS.
//     var ros = new ROSLIB.Ros({
//       url : 'ws://localhost:9090'
//     });

//     // Create the main viewer.
//     var viewer = new ROS2D.Viewer({
//       divID : 'map',
//       width : 800,
//       height : 600
//     });

//     // Setup the map client.
//     var gridClient = new ROS2D.OccupancyGridClient({
//       ros : ros,
//       rootObject : viewer.scene
//     });
//     // Scale the canvas to fit to the map
//     gridClient.on('change', function(){
//       viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
//       viewer.shift(-54.8,-39.8)
//     }); 
//   }

    // Connect to ROS.
    var ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });

    // Create the main viewer.
    var viewer = new ROS2D.Viewer({
      divID : 'map',
      width : 800,
      height : 600
    });

    // Setup the map client.
    var gridClient = new ROS2D.OccupancyGridClient({
      ros : ros,
      rootObject : viewer.scene
    });
    // Scale the canvas to fit to the map
    gridClient.on('change', function(){
      viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
      viewer.shift(-54.8,-39.8)
    }); 

    // Variables to test printing position
    var remainingTime = "0";
    var remainingDistance = "0";
    var batteryState = "0";

    // Topic subscription example on initialpose to use it for other information
    var poseNavigationFeedback = new ROSLIB.Topic({
      ros : ros,
      name : '/camco/navigate_to_pose/feedback_redirected',
      messageType : 'camco_msgs/msg/NavToPoseFeedback'
    });

    var batteryState = new ROSLIB.Topic({
      ros : ros,
      name : '/sensors/battery_state',
      messageType : 'sensor_msgs/msg/BatteryState'
    });

    poseNavigationFeedback.subscribe(function(message) {
      // console.log('Received message on ' + InitialPoseListener.name + ': ' + JSON.stringify(message["pose"]["pose"]["position"]["x"]));
      remainingTime = JSON.stringify(message["estimated_remaining_time.sec"]);
      remainingDistance = JSON.stringify(message["distance_remaining"]);
    });

    batteryState.subscribe(function(message) {
      // console.log('Received message on ' + InitialPoseListener.name + ': ' + JSON.stringify(message["pose"]["pose"]["position"]["x"]));
      batteryState = JSON.stringify(message["percentage"]);
    });
    function updateReadings() {
      // Simulated data for demonstration
      const battery = parseFloat(batteryState); // Random battery state between 0 and 100
      const time = parseFloat(remainingTime);
      const distance = parseFloat(remainingDistance);
      const speed = Math.random() * 100; // Random speed between 0 and 100 km/h
    
      // Format the data into a strings
      const readingsString = `Battery State: ${battery.toFixed(2)}%<br>
                              Remaining Time: ${time.toFixed(2)}m<br>
                              Remaining Distance: ${distance.toFixed(2)}m<br>
                              Speed: ${speed.toFixed(2)} km/h`;
    
      // Update the content of the readings container
      document.getElementById('readings-container').innerHTML = readingsString;
    }
    setInterval(updateReadings, 200);
  const buildings = Object.keys(rooms);
  
  // Function to populate options in a select element
  function populateOptions(selectElement, options) {
    options.forEach(option => {
      const optionElement = document.createElement("option");
      optionElement.value = option;
      optionElement.textContent = option;
      selectElement.appendChild(optionElement);
    });
  }
  
  // Populate options for starting point and destination dropdowns
  const startPointBuilding = document.getElementById("start-point-building");
  const destinationBuilding = document.getElementById("destination-building");
  const startPointRoom = document.getElementById("start-point-room");
  const destinationRoom = document.getElementById("destination-room");

  populateOptions(startPointBuilding, buildings);
  populateOptions(destinationBuilding, buildings);
  populateOptions(startPointRoom, rooms[buildings[0]]);
  populateOptions(destinationRoom, rooms[buildings[0]]);

  startPointBuilding.addEventListener('change', function() {
      const selectedbuilding = this.value;
      const roomDropdown = rooms[selectedbuilding];
      
      // Get destinations based on selected start point
      const roomsSelection = rooms[selectedbuilding];

      // Clear existing options and populate rooms dropdown
      startPointRoom.innerHTML = '';
      populateOptions(startPointRoom, roomsSelection);
    });

    destinationBuilding.addEventListener('change', function() {
      const selectedbuilding = this.value;
      const roomDropdown = rooms[selectedbuilding];
      
      // Get destinations based on selected start point
      const roomsSelection = rooms[selectedbuilding];

      // Clear existing options and populate rooms dropdown
      destinationRoom.innerHTML = '';
      populateOptions(destinationRoom, roomsSelection);
    });

    document.getElementById('navigate-button').addEventListener('click', function() {
      // Create a Publisher object for the topic you want to publish to
      var initialAddress = new ROSLIB.Topic({
        ros: ros,
        name: '/initial_address', // Adjust the topic name according to your ROS setup
        messageType: 'camco_msgs/msg/RoomAddress' // Adjust the message type according to your ROS setup
      });
    
      // Create a message object
      var initial_address = new ROSLIB.Message({
        building: startPointBuilding.value,
        room: startPointRoom.value
      });

      var goalAddress = new ROSLIB.Topic({
        ros: ros,
        name: '/goal_address', // Adjust the topic name according to your ROS setup
        messageType: 'camco_msgs/msg/RoomAddress' // Adjust the message type according to your ROS setup
      });
    
      // Create a message object
      var goal_address = new ROSLIB.Message({
        building: startPointBuilding.value,
        room: startPointRoom.value
      });
    
      // Publish the message to the topic
      initialAddress.publish(initial_address);
      goalAddress.publish(goal_address);
    });
