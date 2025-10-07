% scale(1000) import("r_ankle_mx28_block.stl");

// Sketch PureShapes 51
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, -45.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 51.000000;
translate([0, 0, -thickness]) {
  translate([-21.677526, -20.388570, 0]) {
    rotate([0, 0, 0.0]) {
      cube([84.021384, 43.282660, thickness]);
    }
  }
}
}
