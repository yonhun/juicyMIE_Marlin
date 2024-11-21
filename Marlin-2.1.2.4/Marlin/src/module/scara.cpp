/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * scara.cpp
 */

#include "../inc/MarlinConfig.h"

#if IS_SCARA

#include "scara.h"
#include "motion.h"
#include "planner.h"

#if ANY(PARALLEL_SCARA, AXEL_TPARA)
  #include "endstops.h"
  #include "../MarlinCore.h"
#endif

float segments_per_second = DEFAULT_SEGMENTS_PER_SECOND;

#if ANY(MORGAN_SCARA, MP_SCARA)

  static constexpr xy_pos_t scara_offset = { SCARA_OFFSET_X, SCARA_OFFSET_Y };

  /**
   * Morgan SCARA Forward Kinematics. Results in 'cartes'.
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void forward_kinematics(const_float_t a, const_float_t b) {
    const float a_sin = sin(RADIANS(a)) * L1,
                a_cos = cos(RADIANS(a)) * L1,
                b_sin = sin(RADIANS(SUM_TERN(MP_SCARA, b, a))) * L2,
                b_cos = cos(RADIANS(SUM_TERN(MP_SCARA, b, a))) * L2;

    cartes.x = a_cos + b_cos + scara_offset.x;  // theta
    cartes.y = a_sin + b_sin + scara_offset.y;  // phi

    /*
      DEBUG_ECHOLNPGM(
        "SCARA FK Angle a=", a,
        " b=", b,
        " a_sin=", a_sin,
        " a_cos=", a_cos,
        " b_sin=", b_sin,
        " b_cos=", b_cos
      );
      DEBUG_ECHOLNPGM(" cartes (X,Y) = "(cartes.x, ", ", cartes.y, ")");
    //*/
  }

#endif

#if ENABLED(MORGAN_SCARA)

  void scara_set_axis_is_at_home(const AxisEnum axis) {
    if (axis == Z_AXIS)
      current_position.z = Z_HOME_POS;
    else {
      // MORGAN_SCARA uses a Cartesian XY home position
      xyz_pos_t homeposition = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
      //DEBUG_ECHOLNPGM_P(PSTR("homeposition X"), homeposition.x, SP_Y_LBL, homeposition.y);

      inverse_kinematics(homeposition);
      forward_kinematics(delta.a, delta.b);
      current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPGM_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);
      update_software_endstops(axis);
    }
  }

  /**
   * Morgan SCARA Inverse Kinematics. Results are stored in 'delta'.
   *
   * See https://reprap.org/forum/read.php?185,283327
   *
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void inverse_kinematics(const xyz_pos_t &raw) {
    float C2, S2, SK1, SK2, THETA, PSI;

    // Translate SCARA to standard XY with scaling factor
    const xy_pos_t spos = raw - scara_offset;

    const float H2 = HYPOT2(spos.x, spos.y);
    if (L1 == L2)
      C2 = H2 / L1_2_2 - 1;
    else
      C2 = (H2 - (L1_2 + L2_2)) / (2.0f * L1 * L2);

    LIMIT(C2, -1, 1);

    S2 = SQRT(1.0f - sq(C2));

    // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
    SK1 = L1 + L2 * C2;

    // Rotated Arm2 gives the distance from Arm1 to Arm2
    SK2 = L2 * S2;

    // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
    THETA = ATAN2(SK1, SK2) - ATAN2(spos.x, spos.y);

    // Angle of Arm2
    PSI = ATAN2(S2, C2);

    delta.set(DEGREES(THETA), DEGREES(SUM_TERN(MORGAN_SCARA, PSI, THETA)), raw.z);

    /*
      DEBUG_POS("SCARA IK", raw);
      DEBUG_POS("SCARA IK", delta);
      DEBUG_ECHOLNPGM("  SCARA (x,y) ", sx, ",", sy, " C2=", C2, " S2=", S2, " Theta=", THETA, " Psi=", PSI);
    //*/
  }

#elif ENABLED(MP_SCARA)

  void scara_set_axis_is_at_home(const AxisEnum axis) {
    if (axis == Z_AXIS)
      current_position.z = Z_HOME_POS;
    else {
      // MP_SCARA uses arm angles for AB home position
      #ifndef SCARA_OFFSET_THETA1
        #define SCARA_OFFSET_THETA1  12 // degrees
      #endif
      #ifndef SCARA_OFFSET_THETA2
        #define SCARA_OFFSET_THETA2 131 // degrees
      #endif
      ab_float_t homeposition = { SCARA_OFFSET_THETA1, SCARA_OFFSET_THETA2 };
      //DEBUG_ECHOLNPGM("homeposition A:", homeposition.a, " B:", homeposition.b);

      forward_kinematics(homeposition.a, homeposition.b);
      current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPGM_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);
      update_software_endstops(axis);
    }
  }

  void inverse_kinematics(const xyz_pos_t &raw) {
    const float x = raw.x, y = raw.y, c = HYPOT(x, y),
                THETA3 = ATAN2(y, x),
                THETA1 = THETA3 + ACOS((sq(c) + sq(L1) - sq(L2)) / (2.0f * c * L1)),
                THETA2 = THETA3 - ACOS((sq(c) + sq(L2) - sq(L1)) / (2.0f * c * L2));

    delta.set(DEGREES(THETA1), DEGREES(THETA2), raw.z);

    /*`
      DEBUG_POS("SCARA IK", raw);
      DEBUG_POS("SCARA IK", delta);
      SERIAL_ECHOLNPGM("  SCARA (x,y) ", x, ",", y," Theta1=", THETA1, " Theta2=", THETA2);
    //*/
  }

#elif ENABLED(PARALLEL_SCARA)

  static constexpr xy_pos_t scara_offset = { SCARA_OFFSET_X, SCARA_OFFSET_Y };
  static constexpr float SCARA_OFFSET = SCARA_OFFSET_C;  // 두 암 사이의 오프셋

  void home_PARALLEL_SCARA() {

    // 호밍 시작(엔드스톱 활성화)
    SERIAL_ECHOLNPGM("PARALLEL_SCARA HOMING START");
    endstops.enable(true);  

    // 현재 위치 초기화
    SERIAL_ECHOLNPGM("RESET CURRENT POSITION");
    current_position.reset();
    destination.reset();
    sync_plan_position();

    // 현재 위치 임의 설정
    SERIAL_ECHOLNPGM("SET CURRENT POSITION");
    current_position.set(scara_offset.x + SCARA_OFFSET/2, 0, 0);
    inverse_kinematics(current_position);
    forward_kinematics(delta.a, delta.b);
    sync_plan_position();

    // X축 호밍
    SERIAL_ECHOLNPGM("HOMING X AXIS");
    delta.set(0, delta.b);
    forward_kinematics(delta.a, delta.b);
    destination.x = cartes.x;
    destination.y = cartes.y;
    destination.z = current_position.z;
    prepare_line_to_destination();
    // 엔드스톱 닿으면 현재 X 위치 저장
    if (endstops.trigger_state()) {
      SERIAL_ECHOLNPGM("HIT X ENDSTOP");
      set_axis_is_at_home(X_AXIS);
      sync_plan_position();
      endstops.hit_on_purpose();
    }

    // X축, Y축 안전 위치 이동
    SERIAL_ECHOLNPGM("MOVE TO SAFE POSITION");
    delta.set(180, 45);
    forward_kinematics(delta.a, delta.b);
    prepare_line_to_destination();
    planner.synchronize();
    sync_plan_position();

    // Y축 호밍
    SERIAL_ECHOLNPGM("HOMING Y AXIS");
    delta.set(180, 150);
    forward_kinematics(delta.a, delta.b);
    destination.x = cartes.x;
    destination.y = cartes.y;
    destination.z = current_position.z;
    prepare_line_to_destination();
    if (endstops.trigger_state())
    {
      SERIAL_ECHOLNPGM("HIT Y ENDSTOP");
      set_axis_is_at_home(Y_AXIS);
      sync_plan_position();
      endstops.hit_on_purpose();
    }

    // Z축 호밍
    SERIAL_ECHOLNPGM("HOMING Z AXIS");
    homeaxis(Z_AXIS);

    // 호밍 완료
    endstops.enable(false);
    sync_plan_position();
    SERIAL_ECHOLNPGM("PARALLEL_SCARA HOMING COMPLETE");

    // 0,0 위치로 이동
    SERIAL_ECHOLNPGM("MOVE TO 0,0");
    destination.x = 0;
    destination.y = 0;  
    destination.z = current_position.z;
    prepare_line_to_destination();
    planner.synchronize();
    sync_plan_position();
    SERIAL_ECHOLNPGM("PARALLEL_SCARA MOVE TO 0,0 COMPLETE");
  }

  void scara_set_axis_is_at_home(const AxisEnum axis) {
  if (axis == Z_AXIS) {
    current_position.z = Z_HOME_POS; // Z축 호밍
  } 
  else if (axis == X_AXIS) {
    // X축에 대해 처리
    xyz_pos_t homeposition = { X_HOME_POS, current_position.y, current_position.z };
    inverse_kinematics(homeposition);
    forward_kinematics(delta.a, delta.b);
    current_position.x = cartes.x;
  } 
  else if (axis == Y_AXIS) {
    // Y축에 대해 처리
    xyz_pos_t homeposition = { current_position.x, Y_HOME_POS, current_position.z };
    inverse_kinematics(homeposition);
    forward_kinematics(delta.a, delta.b);
    current_position.y = cartes.y;
  }

  // 소프트웨어 엔드스톱 업데이트
  update_software_endstops(axis);
}


  // PARALLEL_SCARA Forward Kinematics
  void forward_kinematics(const_float_t a, const_float_t b) {
    const float a_rad = RADIANS(a);
    const float b_rad = RADIANS(b);

    // 두 점의 좌표 계산
    float x1 = SCARA_LINKAGE_1 * cos(a_rad);
    float y1 = SCARA_LINKAGE_1 * sin(a_rad);
    float x2 = SCARA_LINKAGE_1 * cos(b_rad) + SCARA_OFFSET;
    float y2 = SCARA_LINKAGE_1 * sin(b_rad);

    // 중점 계산
    float mid_x = (x1 + x2) / 2;
    float mid_y = (y1 + y2) / 2;

    // 두 점 사이의 거리
    float d = HYPOT(x2 - x1, y2 - y1);

    // 중점에서 목표 점까지의 거리
    float h = SQRT(sq(SCARA_LINKAGE_2) - sq(d / 2));

    // 수직 단위 벡터 계산
    float nx, ny;
    if (d != 0) {
      nx = (y1 - y2) / d;
      ny = (x2 - x1) / d;
    } else {
      nx = 0;
      ny = 1;
    }

    // 최종 목표 점 계산
    cartes.x = mid_x + h * nx + scara_offset.x;
    cartes.y = mid_y + h * ny + scara_offset.y;
  }

  // PARALLEL_SCARA Inverse Kinematics
  void inverse_kinematics(const xyz_pos_t &raw) {
    const xy_pos_t spos = raw - scara_offset;
    float target_x = spos.x;
    float target_y = spos.y;

    float a = HYPOT(target_x, target_y);
    float b = HYPOT(target_x - SCARA_OFFSET, target_y);

    float alpha_1 = ACOS((sq(a) + sq(SCARA_OFFSET) - sq(b)) / (2 * a * SCARA_OFFSET));
    float gamma_1 = ACOS((sq(b) + sq(SCARA_OFFSET) - sq(a)) / (2 * b * SCARA_OFFSET));

    float beta_1 = ACOS((sq(SCARA_LINKAGE_1) + sq(a) - sq(SCARA_LINKAGE_2)) / (2 * SCARA_LINKAGE_1 * a));
    float delta_1 = ACOS((sq(SCARA_LINKAGE_1) + sq(b) - sq(SCARA_LINKAGE_2)) / (2 * SCARA_LINKAGE_1 * b));

    float theta_a = DEGREES(alpha_1 + beta_1);
    float theta_b = DEGREES(PI - gamma_1 - delta_1);

    delta.set(theta_a, theta_b, raw.z);
  }

#elif ENABLED(AXEL_TPARA)

  static constexpr xyz_pos_t robot_offset = { TPARA_OFFSET_X, TPARA_OFFSET_Y, TPARA_OFFSET_Z };

  void scara_set_axis_is_at_home(const AxisEnum axis) {
    if (axis == Z_AXIS)
      current_position.z = Z_HOME_POS;
    else {
      xyz_pos_t homeposition = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
      //DEBUG_ECHOLNPGM_P(PSTR("homeposition X"), homeposition.x, SP_Y_LBL, homeposition.y, SP_Z_LBL, homeposition.z);

      inverse_kinematics(homeposition);
      forward_kinematics(delta.a, delta.b, delta.c);
      current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPGM_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);
      update_software_endstops(axis);
    }
  }

  // Convert ABC inputs in degrees to XYZ outputs in mm
  void forward_kinematics(const_float_t a, const_float_t b, const_float_t c) {
    const float w = c - b,
                r = L1 * cos(RADIANS(b)) + L2 * sin(RADIANS(w - (90 - b))),
                x = r  * cos(RADIANS(a)),
                y = r  * sin(RADIANS(a)),
                rho2 = L1_2 + L2_2 - 2.0f * L1 * L2 * cos(RADIANS(w));

    cartes = robot_offset + xyz_pos_t({ x, y, SQRT(rho2 - sq(x) - sq(y)) });
  }

  // Home YZ together, then X (or all at once). Based on quick_home_xy & home_delta
  void home_TPARA() {
    // Init the current position of all carriages to 0,0,0
    current_position.reset();
    destination.reset();
    sync_plan_position();

    // Disable stealthChop if used. Enable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      TERN_(X_SENSORLESS, sensorless_t stealth_states_x = start_sensorless_homing_per_axis(X_AXIS));
      TERN_(Y_SENSORLESS, sensorless_t stealth_states_y = start_sensorless_homing_per_axis(Y_AXIS));
      TERN_(Z_SENSORLESS, sensorless_t stealth_states_z = start_sensorless_homing_per_axis(Z_AXIS));
    #endif

    //const int x_axis_home_dir = TOOL_X_HOME_DIR(active_extruder);

    //const xy_pos_t pos { max_length(X_AXIS) , max_length(Y_AXIS) };
    //const float mlz = max_length(X_AXIS),

    // Move all carriages together linearly until an endstop is hit.
    //do_blocking_move_to_xy_z(pos, mlz, homing_feedrate(Z_AXIS));

    current_position.set(0, 0, max_length(Z_AXIS));
    line_to_current_position(homing_feedrate(Z_AXIS));
    planner.synchronize();

    // Re-enable stealthChop if used. Disable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      TERN_(X_SENSORLESS, end_sensorless_homing_per_axis(X_AXIS, stealth_states_x));
      TERN_(Y_SENSORLESS, end_sensorless_homing_per_axis(Y_AXIS, stealth_states_y));
      TERN_(Z_SENSORLESS, end_sensorless_homing_per_axis(Z_AXIS, stealth_states_z));
    #endif

    endstops.validate_homing_move();

    // At least one motor has reached its endstop.
    // Now re-home each motor separately.
    homeaxis(A_AXIS);
    homeaxis(C_AXIS);
    homeaxis(B_AXIS);

    // Set all carriages to their home positions
    // Do this here all at once for Delta, because
    // XYZ isn't ABC. Applying this per-tower would
    // give the impression that they are the same.
    LOOP_NUM_AXES(i) set_axis_is_at_home((AxisEnum)i);

    sync_plan_position();
  }

  void inverse_kinematics(const xyz_pos_t &raw) {
    const xyz_pos_t spos = raw - robot_offset;

    const float RXY = SQRT(HYPOT2(spos.x, spos.y)),
                RHO2 = NORMSQ(spos.x, spos.y, spos.z),
                //RHO = SQRT(RHO2),
                LSS = L1_2 + L2_2,
                LM = 2.0f * L1 * L2,

                CG = (LSS - RHO2) / LM,
                SG = SQRT(1 - POW(CG, 2)), // Method 2
                K1 = L1 - L2 * CG,
                K2 = L2 * SG,

                // Angle of Body Joint
                THETA = ATAN2(spos.y, spos.x),

                // Angle of Elbow Joint
                //GAMMA = ACOS(CG),
                GAMMA = ATAN2(SG, CG), // Method 2

                // Angle of Shoulder Joint, elevation angle measured from horizontal (r+)
                //PHI = asin(spos.z/RHO) + asin(L2 * sin(GAMMA) / RHO),
                PHI = ATAN2(spos.z, RXY) + ATAN2(K2, K1),   // Method 2

                // Elbow motor angle measured from horizontal, same frame as shoulder  (r+)
                PSI = PHI + GAMMA;

    delta.set(DEGREES(THETA), DEGREES(PHI), DEGREES(PSI));

    //SERIAL_ECHOLNPGM(" SCARA (x,y,z) ", spos.x , ",", spos.y, ",", spos.z, " Rho=", RHO, " Rho2=", RHO2, " Theta=", THETA, " Phi=", PHI, " Psi=", PSI, " Gamma=", GAMMA);
  }
#endif

void scara_report_positions() {
  SERIAL_ECHOLNPGM("SCARA Theta:", planner.get_axis_position_degrees(A_AXIS)
    #if ENABLED(AXEL_TPARA)
      , "  Phi:", planner.get_axis_position_degrees(B_AXIS)
      , "  Psi:", planner.get_axis_position_degrees(C_AXIS)
    #else
      , "  Psi" TERN_(MORGAN_SCARA, "+Theta") ":", planner.get_axis_position_degrees(B_AXIS)
    #endif
  );
  SERIAL_EOL();
}

#endif // IS_SCARA
