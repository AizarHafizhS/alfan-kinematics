% scale(1000) import("r_shoulder_connector.stl");

// Sketch PureShapes 29
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 29.000000;
translate([0, 0, -thickness]) {
  translate([-11.069545, -18.023908, 0]) {
    rotate([0, 0, 0.0]) {
      cube([38.152068, 36.059659, thickness]);
    }
  }
}
}
