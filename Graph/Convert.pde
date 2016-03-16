//convert all axis
final int minAngle = -180;
final int maxAngle = 180;

void convert() {

  /* Convert the kalman filter x-axis */
  if (stringYaw != null) {
    stringYaw = trim(stringYaw); // Trim off any whitespace
    yaw[yaw.length - 1] = map(float(stringYaw), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

}
