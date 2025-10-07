% scale(1000) import("l_shoulder_connector.stl");

// Sketch PureShapes 29
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 29.000000;
translate([0, 0, -thickness]) {
  translate([-11.064615, -18.004633, 0]) {
    rotate([0, 0, 0.0]) {
      cube([38.113013, 35.999790, thickness]);
    }
  }
}
}
