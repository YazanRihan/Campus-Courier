const rooms = {
  "M8":
    ["main", "001", "002", "003", "004", "005", "007", "008", "009", "bathroom", "010", "011", "012"],

  "M9":
    ["main", "001", "002", "005", "006", "007", "008", "bathroom", "009", "010", "011"],

  "M10":
    ["main", "001", "002", "003", "004", "005", "006", "007", "008", "bathroom", "009", "010", "011"]
}

function init() {
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
  }

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
