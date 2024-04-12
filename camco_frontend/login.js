// Select the login button
const loginButton = document.getElementById("enteredButton");

// Add click event listener to the login button
loginButton.addEventListener("click", function(event) {
  // Prevent the default form submission behavior
  event.preventDefault();
  
  // Redirect the user to "mission.html"
  window.location.href = "mission.html";
});
