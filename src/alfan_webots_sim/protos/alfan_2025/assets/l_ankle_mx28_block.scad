% scale(1000) import("l_ankle_mx28_block.stl");

// Sketch PureShapes 51
multmatrix([[1.0, 0.0, -3.46944695195361e-18, -1.5612511283791244e-16], [0.0, -1.0, 0.0, 0.0], [-3.46944695195361e-18, 0.0, -1.0, -45.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 51.000000;
translate([0, 0, -thickness]) {
  translate([-21.640530, -22.961868, 0]) {
    rotate([0, 0, 0.0]) {
      cube([84.154485, 43.396968, thickness]);
    }
  }
}
}
