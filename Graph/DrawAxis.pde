void drawAxisX() {
  /* Draw yaw x-axis */
  noFill();
  stroke(255, 0, 0);// Red
  // Redraw everything
  beginShape();
  vertex(0, yaw[0]);
  for (int i = 1; i < yaw.length; i++) {
    if ((yaw[i] < height/4 && yaw[i - 1] > height/4*3) || (yaw[i] > height/4*3 && yaw[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, yaw[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < yaw.length; i++)
    yaw[i-1] = yaw[i];
}

