// Orbit math of the orbit-camera mod: pure functions, no game access, no SDL.
//
// The game folds view x model on the CPU and uploads the product per object as the WORLD matrix (docs/FACTS.md of
// gran-turismo-decomp, "camera"): 16 floats, row vectors, rows 0..2 the basis, row 3 the translation, x_view = [x 1] * M.
// An orbit of the camera around a pivot p (a point in VIEW space) is then M' = M * T(-p) * R_y(yaw) * R_x(pitch) * T(p).
#pragma once

namespace orbit {

// kYawSignNote: R_y is the spike's matrix {c,0,s; 0,1,0; -s,0,c} (row vectors). Positive yaw swings the camera to the car's
// right side (the car is seen from its right), verified by frames (docs/FACTS.md "orbit camera"); the stick mapping uses
// +90 for stick right.
void orbit_matrix(const float m[16], const float pivot[3], float yaw_deg, float pitch_deg, float out[16]);

// The projection matrix of a perspective (3D) upload has m[11] = -1; the HUD's orthographic one has 0.
bool is_perspective(float proj_m11);

// Car position in the game's Z-up form (x, y, z) -> the view's Y-up form (x, z, -y).
void car_to_yup(float x, float y, float z, float out[3]);

// The pivot in view space: p[r] = dot(row_r of the camera-to-world matrix, carpos - eye) for r = 0..2.
void pivot_in_view(const float cam_to_world[16], const float carpos_yup[3], float out[3]);

// No write at all (frames byte-identical to a run without the mod) when the orbit is the identity.
bool needs_edit(float yaw_deg, float pitch_deg);

}  // namespace orbit
