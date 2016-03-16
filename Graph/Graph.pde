import processing.serial.*;
Serial serial;

String stringYaw;
String time;

final int width = 800;
final int height = 600;

float[] yaw = new float[width];

boolean drawValues  = false;

PrintWriter output; //SAM

void setup() {
  size(width, height);
  println(Serial.list()); // Use this to print connected serial devices
  serial = new Serial(this, Serial.list()[3], 115200); // Set this to your serial port obtained using the line above
  serial.bufferUntil('\n'); // Buffer until line feed

  for (int i = 0; i < width; i++) { // center all variables
    yaw[i] = height/2;
  }

  drawGraph(); // Draw graph at startup
  output = createWriter("Control_Data.txt"); //SAM
}

void draw() {
  /* Draw Graph */
  if (drawValues) {
    drawValues = false;
    drawGraph();
  }
  output.println(stringYaw); //SAM
}

void drawGraph() {
  background(255); // White
  for (int i = 0; i < width; i++) {
    stroke(200); // Grey
    line(i*10, 0, i*10, height);
    line(0, i*10, width, i*10);
  }

  stroke(0); // Black
  for (int i = 1; i <= 3; i++)
    line(0, height/4*i, width, height/4*i); // Draw line, indicating -90 deg, 0 deg and 90 deg

  convert();
  drawAxisX();
  drawAxisY();
}

void serialEvent (Serial serial) {
  // Get the ASCII strings:
  stringYaw = serial.readStringUntil('\t');

  serial.clear(); // Clear buffer
  drawValues = true; // Draw the graph

  //printAxis(); // Used for debugging
}

void printAxis() {
  print(stringYaw);

  println();
}

void keyPressed() { //SAM
  output.flush();  // Writes the remaining data to the file
  output.close();  // Finishes the file
  exit();  // Stops the program
}
