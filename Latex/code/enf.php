<?php
$servername = "pi";
$username = "*****";
$password = "********";

// Create connection
$conn = new mysqli($servername, $username, $password, "sysm");

// Check connection
if ($conn->connect_error) {
  die("Connection failed: " . $conn->connect_error);}
echo "Connected successfully";

date_default_timezone_set("Europe/Berlin");
$timestamp = time();
$entfernung = intval($_GET['enf']);
echo "$entfernung";
echo "<br />";
$date = date("Y-m-d H:i:s", $timestamp);
$sql = "INSERT INTO lidar(entfernung, timestamp) VALUES ($entfernung, '$date')";

if ($conn->query($sql) === TRUE) {
  echo "New record created successfully";
} else {
  echo "Error: " . $sql . "<br>" . $conn->error;}

$conn->close();
?>

