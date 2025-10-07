% scale(1000) import("l_hip_mx28_block.stl");

// Sketch PureShapes 51
multmatrix([[1.0, 0.0, 3.46944695195361e-18, 9.614931805159e-32], [0.0, 1.0, 0.0, 0.0], [-3.46944695195361e-18, 0.0, 1.0, 2.771315410874039e-14], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 51.000000;
translate([0, 0, -thickness]) {
  translate([-23.684384, -20.086255, 0]) {
    rotate([0, 0, 0.0]) {
      cube([86.005265, 42.263243, thickness]);
    }
  }
}
}
