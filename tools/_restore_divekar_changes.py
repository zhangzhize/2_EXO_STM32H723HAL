"""Restore all Divekar-related changes lost by git checkout."""
with open('../Core/UserInc/exo.hpp', 'r', encoding='utf-8') as f: h = f.read()
with open('../Core/UserSrc/exo.cpp', 'r', encoding='utf-8') as f: c = f.read()

# ════════════ 1. DivekarParams additions ════════════
# Add grf_dot_lpf_alpha and torque_lpf_alpha
h = h.replace(
    '    float output_extension_sign = 1.0f;',
    '    float output_extension_sign = 1.0f;\n\n'
    '    float grf_dot_lpf_alpha = 0.1f; // EMA alpha for GRF derivative LPF\n'
    '    float torque_lpf_alpha = 0.147f; // EMA alpha for torque LPF, 200Hz fc~5Hz')

# ════════════ 2. Remove PlanarLegGeometry (merge into DivekarState) ════════════
import re
# Remove leg_geometry_ struct and member
m = re.search(r'  struct PlanarLegGeometry\n  \{(.*?)\} leg_geometry_;\n\n', h, re.DOTALL)
if m:
    h = h.replace(m.group(0), '')

# Add ankle_x/y_m to DivekarState (theta_la_rad already there)
h = h.replace(
    '    float theta_la_rad = 0.0f;  /*!< positive when hip is anterior to ankle */',
    '    float theta_la_rad = 0.0f;  /*!< positive when hip is anterior to ankle */\n'
    '    float ankle_x_m = 0.0f;\n'
    '    float ankle_y_m = 0.0f;')

# ════════════ 3. DivekarState additions ════════════
# Add LPF state, dt_s, and gate fields to DivekarState
h = h.replace(
    '    bool has_prev = true;\n  } divekar_state_;',
    '\n    // LPF state for GRF dot\n'
    '    float f_grf_ipsi_BW_prev = 0.0f;\n'
    '    float f_grf_ipsi_BW_dot_lpf = 0.0f;\n'
    '    float tau_divekar_lpf_prev_Nm = 0.0f;\n'
    '    float delta_ajc_y_fs_cm = 0.0f;\n'
    '    float delta_ajc_dist_fs_cm = 0.0f;\n'
    '    float gate_delta_ajc_y_a = 0.0f;\n'
    '    float gate_delta_ajc_y_na = 0.0f;\n'
    '    float gate_delta_ajc_dist = 0.0f;\n'
    '    bool is_leading_leg = false;\n'
    '    bool has_prev = true;\n'
    '  } divekar_state_;')

# ════════════ 4. DivekarOutput additions ════════════
# Rename tau fields and add gates
out_old = '    float tau_raw_Nm = 0.0f;\n    float tau_limited_Nm = 0.0f;\n    float tau_motor_cmd_Nm = 0.0f;'
out_new = ('    // --- torques ---\n'
           '    float tau_a_Nm = 0.0f;\n    float tau_na_Nm = 0.0f;\n    float tau_LL_Nm = 0.0f;\n    float tau_grav_st_Nm = 0.0f;\n'
           '    float tau_a_mod_Nm = 0.0f;\n    float tau_na_mod_Nm = 0.0f;\n    float tau_LL_mod_Nm = 0.0f;\n    float tau_grav_st_mod_Nm = 0.0f;\n'
           '    float tau_st_Nm = 0.0f;\n    float tau_grav_sw_Nm = 0.0f;\n    float tau_inertial_sw_Nm = 0.0f;\n    float tau_sd_sw_Nm = 0.0f;\n    float tau_sw_Nm = 0.0f;\n'
           '    float alpha_stance = 0.0f;\n'
           '    float tau_divekar_Nm = 0.0f;\n    float tau_divekar_lpf_Nm = 0.0f;\n    float tau_divekar_lpf_limited_Nm = 0.0f;\n'
           '    // --- kinematics ---\n'
           '    float theta_k_hs_rad = 0.0f;\n    float theta_kd_rad = 0.0f;\n    float theta_kd_max_rad = 0.0f;\n    float theta_k_ddot_used_radps2 = 0.0f;\n'
           '    // --- gates ---\n'
           '    float gate_theta_la = 0.0f;\n    float gate_theta_kd_max = 0.0f;\n    float gate_theta_k_LL_nested = 0.0f;\n    float gate_theta_k_dot_LL = 0.0f;\n'
           '    float gate_F_heel_ipsi = 0.0f;\n    float gate_F_heel_contra = 0.0f;\n    float gate_delta_ajc_dist = 0.0f;\n'
           '    float gate_delta_ajc_dist_inv = 0.0f;\n    float gate_delta_ajc_y_a = 0.0f;\n    float gate_delta_ajc_y_na = 0.0f;\n'
           '    float gate_delta_ajc_dist_plus_F_grf = 0.0f;\n    float gate_F_heel_contra_inv = 0.0f;\n    float gate_F_grf_contra_inv = 0.0f;\n'
           '    float gate_theta_k_dot_sw = 0.0f;\n    float gate_F_grf_ipsi = 0.0f;\n'
           '    float gate_sts_seat = 0.0f;\n    float gate_sts_trunk = 0.0f;\n    float gate_sts_load_rise = 0.0f;\n    float gate_sts_thigh_rise = 0.0f;\n'
           '    float gate_sts_knee_lower_vel = 0.0f;\n    float gate_sts_sit2stand = 0.0f;\n    float gate_sts_stand2sit = 0.0f;\n    float gate_sts_ctx = 0.0f;\n'
           '    float tau_sit2stand_Nm = 0.0f;\n    float tau_stand2sit_Nm = 0.0f;\n    float tau_sts_Nm = 0.0f;\n    float tau_sts_mod_Nm = 0.0f;')
h = h.replace(out_old, out_new)

# ════════════ 5. BiLegContex: add gate_sts_knee_pos_sym, remove old ════════════
# gate_sts_knee_pos_sym computation uses theta_k_abs_diff_rad which is in BiLegContex
# Keep it simple - add to BiLegContex
h = h.replace(
    '    float gate_F_grf_inv_right = 0.0f;',
    '    float gate_F_grf_inv_right = 0.0f;\n'
    '    float gate_sts_knee_pos_sym = 0.0f;')

# ════════════ 6. Add function declarations ════════════
h = h.replace(
    '  void DivekarUpdate();',
    '  void DivekarUpdate();\n'
    '  static void UpdateDivekarBiLegContext(KneeJoint &left_knee, KneeJoint &right_knee, const ExoData &pe);\n'
    '  static void LatchDivekarLeadingLeg(KneeJoint &left_knee, KneeJoint &right_knee, bool left_fs, bool right_fs);')
h = h.replace(
    '  float GetThetaKddot(const DivekarState &in, float dt_s);',
    '  float GetThetaKddot(const DivekarState &in, float dt_s);\n'
    '  float GetGrfDotLpf(float f_sum_grf_bw, float dt_s);\n'
    '  float GetTorqueLpf(float tau_unfiltered_Nm);')
h = h.replace(
    '  void ComputeAnkleGeometry();',
    '  void ComputeAnkleGeometry();\n'
    '  static void ComputeDeltaAjcYLeftMinusRight(const DivekarState &left, const DivekarState &right);\n'
    '  static void ComputeDeltaAjcDist(const DivekarState &left, const DivekarState &right);')
# Rename ComputePlanarLegGeometry → ComputeAnkleGeometry
h = h.replace('ComputePlanarLegGeometry', 'ComputeAnkleGeometry')

print('Header done')

# ════════════ 7. CPP: Core structural changes ════════════

# Remove theta_la_rad copy (was: divekar_state_.theta_la_rad = leg_geometry_.theta_la_rad)
c = c.replace('  divekar_state_.theta_la_rad = leg_geometry_.theta_la_rad;\n', '')

# Replace leg_geometry_ references with divekar_state_ in ComputePlanarLegGeometry
c = c.replace('leg_geometry_.ankle', 'divekar_state_.ankle')
c = c.replace('leg_geometry_.theta_la_rad', 'divekar_state_.theta_la_rad')

# Rename ComputePlanarLegGeometry -> ComputeAnkleGeometry
c = c.replace('void KneeJoint::ComputePlanarLegGeometry', 'void KneeJoint::ComputeAnkleGeometry')
c = c.replace('ComputePlanarLegGeometry', 'ComputeAnkleGeometry')

# Fix static function signatures and bodies
c = c.replace(
    'void KneeJoint::ComputeDeltaAjcY(const PlanarLegGeometry &leading, const PlanarLegGeometry &trailing)\n',
    'void KneeJoint::ComputeDeltaAjcYLeftMinusRight(const DivekarState &left, const DivekarState &right)\n    // simplified: just Y difference\n')
c = c.replace(
    'void KneeJoint::ComputeDeltaAjcDist(const PlanarLegGeometry &left, const PlanarLegGeometry &right)\n',
    'void KneeJoint::ComputeDeltaAjcDist(const DivekarState &left, const DivekarState &right)\n')

# Fix caller: replace leg_geometry_ with divekar_state_ in caller
c = c.replace('left_side_.knee_joint_.leg_geometry_', 'left_side_.knee_joint_.divekar_state_')
c = c.replace('right_side_.knee_joint_.leg_geometry_', 'right_side_.knee_joint_.divekar_state_')

# Fix old ComputeDeltaAjcY/Dist body references
c = c.replace('leading.ankle_y_m - trailing.ankle_y_m', 'left.ankle_y_m - right.ankle_y_m')
c = c.replace('bi_leg_geometry_.delta_ajc_y_cm', 'bi_leg_ctx_.delta_ajc_y_l_minus_r_cm')
c = c.replace('bi_leg_geometry_.delta_ajc_dist_cm', 'bi_leg_ctx_.delta_ajc_dist_cm')

print('CPP done')
with open('../Core/UserInc/exo.hpp', 'w', encoding='utf-8') as f: f.write(h)
with open('../Core/UserSrc/exo.cpp', 'w', encoding='utf-8') as f: f.write(c)
print('All done - check for compilation errors')
