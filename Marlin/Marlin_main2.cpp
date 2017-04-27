/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
inline void gcode_G4() {
  millis_t dwell_ms = 0;

  if (code_seen('P')) dwell_ms = code_value_millis(); // milliseconds to wait
  if (code_seen('S')) dwell_ms = code_value_millis_from_seconds(); // seconds to wait

  stepper.synchronize();
  refresh_cmd_timeout();
  dwell_ms += previous_cmd_ms;  // keep track of when we started waiting

  if (!lcd_hasstatus()) LCD_MESSAGEPGM(MSG_DWELL);

  while (PENDING(millis(), dwell_ms)) idle();
}

//&begin[Arc_Movement]
#if ENABLED(BEZIER_CURVE_SUPPORT)

  /**
   * Parameters interpreted according to:
   * http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G5-Cubic-Spline
   * However I, J omission is not supported at this point; all
   * parameters can be omitted and default to zero.
   */

  /**
   * G5: Cubic B-spline
   */
  inline void gcode_G5() {
    if (IsRunning()) {

      gcode_get_destination();

      float offset[] = {
        code_seen('I') ? code_value_axis_units(X_AXIS) : 0.0,
        code_seen('J') ? code_value_axis_units(Y_AXIS) : 0.0,
        code_seen('P') ? code_value_axis_units(X_AXIS) : 0.0,
        code_seen('Q') ? code_value_axis_units(Y_AXIS) : 0.0
      };

      plan_cubic_move(offset);
    }
  }

#endif // BEZIER_CURVE_SUPPORT
//&end[Arc_Movement]

#if ENABLED(FWRETRACT)

  /**
   * G10 - Retract filament according to settings of M207
   * G11 - Recover filament according to settings of M208
   */
  inline void gcode_G10_G11(bool doRetract=false) {
    #if EXTRUDERS > 1
      if (doRetract) {
        retracted_swap[active_extruder] = (code_seen('S') && code_value_bool()); // checks for swap retract argument
      }
    #endif
    retract(doRetract
     #if EXTRUDERS > 1
      , retracted_swap[active_extruder]
     #endif
    );
  }

#endif //FWRETRACT

#if ENABLED(NOZZLE_CLEAN_FEATURE)
  /**
   * G12: Clean the nozzle
   */
  inline void gcode_G12() {
    // Don't allow nozzle cleaning without homing first
    if (axis_unhomed_error(true, true, true)) { return; }

    uint8_t const pattern = code_seen('P') ? code_value_ushort() : 0;
    uint8_t const strokes = code_seen('S') ? code_value_ushort() : NOZZLE_CLEAN_STROKES;
    uint8_t const objects = code_seen('T') ? code_value_ushort() : 3;

    Nozzle::clean(pattern, strokes, objects);
  }
#endif

//&begin[Inch_Mode_Support]
#if ENABLED(INCH_MODE_SUPPORT)
  /**
   * G20: Set input mode to inches
   */
  inline void gcode_G20() { set_input_linear_units(LINEARUNIT_INCH); }

  /**
   * G21: Set input mode to millimeters
   */
  inline void gcode_G21() { set_input_linear_units(LINEARUNIT_MM); }
#endif
//&end[Inch_Mode_Support]

#if ENABLED(NOZZLE_PARK_FEATURE)
  /**
   * G27: Park the nozzle
   */
  inline void gcode_G27() {
    // Don't allow nozzle parking without homing first
    if (axis_unhomed_error(true, true, true)) { return; }
    uint8_t const z_action = code_seen('P') ? code_value_ushort() : 0;
    Nozzle::park(z_action);
  }
#endif // NOZZLE_PARK_FEATURE

//&begin[Homing]
#if ENABLED(QUICK_HOME)

  static void quick_home_xy() {

    // Pretend the current position is 0,0
    current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;
    sync_plan_position();

    int x_axis_home_dir =
      #if ENABLED(DUAL_X_CARRIAGE)
        x_home_dir(active_extruder)
      #else
        home_dir(X_AXIS)
      #endif
    ;

    float mlx = max_length(X_AXIS),
          mly = max_length(Y_AXIS),
          mlratio = mlx > mly ? mly / mlx : mlx / mly,
          fr_mm_s = min(homing_feedrate_mm_s[X_AXIS], homing_feedrate_mm_s[Y_AXIS]) * sqrt(sq(mlratio) + 1.0);

    do_blocking_move_to_xy(1.5 * mlx * x_axis_home_dir, 1.5 * mly * home_dir(Y_AXIS), fr_mm_s);
    endstops.hit_on_purpose(); // clear endstop hit flags
    current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;

  }

#endif // QUICK_HOME
//&end[Homing]

#if ENABLED(DEBUG_LEVELING_FEATURE)

  void log_machine_info() {
    SERIAL_ECHOPGM("Machine Type: ");
    #if ENABLED(DELTA)
      SERIAL_ECHOLNPGM("Delta");
    #elif IS_SCARA
      SERIAL_ECHOLNPGM("SCARA");
    #elif IS_CORE
      SERIAL_ECHOLNPGM("Core");
    #else
      SERIAL_ECHOLNPGM("Cartesian");
    #endif

    SERIAL_ECHOPGM("Probe: ");
    #if ENABLED(FIX_MOUNTED_PROBE)
      SERIAL_ECHOLNPGM("FIX_MOUNTED_PROBE");
    #elif ENABLED(BLTOUCH)
      SERIAL_ECHOLNPGM("BLTOUCH");
    #elif HAS_Z_SERVO_ENDSTOP
      SERIAL_ECHOLNPGM("SERVO PROBE");
    #elif ENABLED(Z_PROBE_SLED)
      SERIAL_ECHOLNPGM("Z_PROBE_SLED");
    #elif ENABLED(Z_PROBE_ALLEN_KEY)
      SERIAL_ECHOLNPGM("Z_PROBE_ALLEN_KEY");
    #else
      SERIAL_ECHOLNPGM("NONE");
    #endif

    #if HAS_BED_PROBE
      SERIAL_ECHOPAIR("Probe Offset X:", X_PROBE_OFFSET_FROM_EXTRUDER);
      SERIAL_ECHOPAIR(" Y:", Y_PROBE_OFFSET_FROM_EXTRUDER);
      SERIAL_ECHOPAIR(" Z:", zprobe_zoffset);
      #if (X_PROBE_OFFSET_FROM_EXTRUDER > 0)
        SERIAL_ECHOPGM(" (Right");
      #elif (X_PROBE_OFFSET_FROM_EXTRUDER < 0)
        SERIAL_ECHOPGM(" (Left");
      #elif (Y_PROBE_OFFSET_FROM_EXTRUDER != 0)
        SERIAL_ECHOPGM(" (Middle");
      #else
        SERIAL_ECHOPGM(" (Aligned With");
      #endif
      #if (Y_PROBE_OFFSET_FROM_EXTRUDER > 0)
        SERIAL_ECHOPGM("-Back");
      #elif (Y_PROBE_OFFSET_FROM_EXTRUDER < 0)
        SERIAL_ECHOPGM("-Front");
      #elif (X_PROBE_OFFSET_FROM_EXTRUDER != 0)
        SERIAL_ECHOPGM("-Center");
      #endif
      if (zprobe_zoffset < 0)
        SERIAL_ECHOPGM(" & Below");
      else if (zprobe_zoffset > 0)
        SERIAL_ECHOPGM(" & Above");
      else
        SERIAL_ECHOPGM(" & Same Z as");
      SERIAL_ECHOLNPGM(" Nozzle)");
    #endif

    #if HAS_ABL
      SERIAL_ECHOPGM("Auto Bed Leveling: ");
      #if ENABLED(AUTO_BED_LEVELING_LINEAR)
        SERIAL_ECHOPGM("LINEAR");
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        SERIAL_ECHOPGM("BILINEAR");
      #elif ENABLED(AUTO_BED_LEVELING_3POINT)
        SERIAL_ECHOPGM("3POINT");
      #endif
      if (planner.abl_enabled) {
        SERIAL_ECHOLNPGM(" (enabled)");
        #if ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(AUTO_BED_LEVELING_3POINT)
          float diff[XYZ] = {
            stepper.get_axis_position_mm(X_AXIS) - current_position[X_AXIS],
            stepper.get_axis_position_mm(Y_AXIS) - current_position[Y_AXIS],
            stepper.get_axis_position_mm(Z_AXIS) - current_position[Z_AXIS]
          };
          SERIAL_ECHOPGM("ABL Adjustment X");
          if (diff[X_AXIS] > 0) SERIAL_CHAR('+');
          SERIAL_ECHO(diff[X_AXIS]);
          SERIAL_ECHOPGM(" Y");
          if (diff[Y_AXIS] > 0) SERIAL_CHAR('+');
          SERIAL_ECHO(diff[Y_AXIS]);
          SERIAL_ECHOPGM(" Z");
          if (diff[Z_AXIS] > 0) SERIAL_CHAR('+');
          SERIAL_ECHO(diff[Z_AXIS]);
        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
          SERIAL_ECHOPAIR("ABL Adjustment Z", bilinear_z_offset(current_position));
        #endif
      }
      SERIAL_EOL;
    #elif ENABLED(MESH_BED_LEVELING)
      SERIAL_ECHOPGM("Mesh Bed Leveling");
      if (mbl.active()) {
        float lz = current_position[Z_AXIS];
        planner.apply_leveling(current_position[X_AXIS], current_position[Y_AXIS], lz);
        SERIAL_ECHOLNPGM(" (enabled)");
        SERIAL_ECHOPAIR("MBL Adjustment Z", lz);
      }
      SERIAL_EOL;
    #endif

  }

#endif // DEBUG_LEVELING_FEATURE

#if ENABLED(DELTA)

  /**
   * A delta can only safely home all axes at the same time
   * This is like quick_home_xy() but for 3 towers.
   */
  inline void home_delta() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS(">>> home_delta", current_position);
    #endif
    // Init the current position of all carriages to 0,0,0
    ZERO(current_position);
    sync_plan_position();

    // Move all carriages together linearly until an endstop is hit.
    current_position[X_AXIS] = current_position[Y_AXIS] = current_position[Z_AXIS] = (Z_MAX_LENGTH + 10);
    feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
    line_to_current_position();
    stepper.synchronize();
    endstops.hit_on_purpose(); // clear endstop hit flags

    // At least one carriage has reached the top.
    // Now re-home each carriage separately.
    HOMEAXIS(A);
    HOMEAXIS(B);
    HOMEAXIS(C);

    // Set all carriages to their home positions
    // Do this here all at once for Delta, because
    // XYZ isn't ABC. Applying this per-tower would
    // give the impression that they are the same.
    LOOP_XYZ(i) set_axis_is_at_home((AxisEnum)i);

    SYNC_PLAN_POSITION_KINEMATIC();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("<<< home_delta", current_position);
    #endif
  }

#endif // DELTA

#if ENABLED(Z_SAFE_HOMING)

  inline void home_z_safely() {

    // Disallow Z homing if X or Y are unknown
    if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
      LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_ERR_Z_HOMING);
      return;
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Z_SAFE_HOMING >>>");
    #endif

    SYNC_PLAN_POSITION_KINEMATIC();

    /**
     * Move the Z probe (or just the nozzle) to the safe homing point
     */
    destination[X_AXIS] = LOGICAL_X_POSITION(Z_SAFE_HOMING_X_POINT);
    destination[Y_AXIS] = LOGICAL_Y_POSITION(Z_SAFE_HOMING_Y_POINT);
    destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

    if (position_is_reachable(
          destination
          #if HOMING_Z_WITH_PROBE
            , true
          #endif
        )
    ) {

      #if HOMING_Z_WITH_PROBE
        destination[X_AXIS] -= X_PROBE_OFFSET_FROM_EXTRUDER;
        destination[Y_AXIS] -= Y_PROBE_OFFSET_FROM_EXTRUDER;
      #endif

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("Z_SAFE_HOMING", destination);
      #endif

      // This causes the carriage on Dual X to unpark
      #if ENABLED(DUAL_X_CARRIAGE)
        active_extruder_parked = false;
      #endif

      do_blocking_move_to_xy(destination[X_AXIS], destination[Y_AXIS]);
      HOMEAXIS(Z);
    }
    else {
      LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< Z_SAFE_HOMING");
    #endif
  }

#endif // Z_SAFE_HOMING

//&begin[Homing]
/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 */
inline void gcode_G28() {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOLNPGM(">>> gcode_G28");
      log_machine_info();
    }
  #endif

  // Wait for planner moves to finish!
  stepper.synchronize();

  // For auto bed leveling, clear the level matrix
  #if HAS_ABL
    reset_bed_level();
  #endif

  // Always home with tool 0 active
  #if HOTENDS > 1
    uint8_t old_tool_index = active_extruder;
    tool_change(0, 0, true);
  #endif

  #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
    extruder_duplication_enabled = false;
  #endif

  /**
   * For mesh bed leveling deactivate the mesh calculations, will be turned
   * on again when homing all axis
   */
  #if ENABLED(MESH_BED_LEVELING)
    float pre_home_z = MESH_HOME_SEARCH_Z;
    if (mbl.active()) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("MBL was active");
      #endif
      // Use known Z position if already homed
      if (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS]) {
        set_bed_leveling_enabled(false);
        pre_home_z = current_position[Z_AXIS];
      }
      else {
        mbl.set_active(false);
        current_position[Z_AXIS] = pre_home_z;
      }
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("Set Z to pre_home_z", current_position);
      #endif
    }
  #endif

  setup_for_endstop_or_probe_move();
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("> endstops.enable(true)");
  #endif
  endstops.enable(true); // Enable endstops for next homing move

  #if ENABLED(DELTA)

    home_delta();

  #else // NOT DELTA

    bool homeX = code_seen('X'), homeY = code_seen('Y'), homeZ = code_seen('Z');

    home_all_axis = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

    set_destination_to_current();

    #if Z_HOME_DIR > 0  // If homing away from BED do Z first

      if (home_all_axis || homeZ) {
        HOMEAXIS(Z);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> HOMEAXIS(Z)", current_position);
        #endif
      }

    #else

      if (home_all_axis || homeX || homeY) {
        // Raise Z before homing any other axes and z is not already high enough (never lower z)
        destination[Z_AXIS] = LOGICAL_Z_POSITION(Z_HOMING_HEIGHT);
        if (destination[Z_AXIS] > current_position[Z_AXIS]) {

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING))
              SERIAL_ECHOLNPAIR("Raise Z (before homing) to ", destination[Z_AXIS]);
          #endif

          do_blocking_move_to_z(destination[Z_AXIS]);
        }
      }

    #endif

    #if ENABLED(QUICK_HOME)

      if (home_all_axis || (homeX && homeY)) quick_home_xy();

    #endif

    #if ENABLED(HOME_Y_BEFORE_X)

      // Home Y
      if (home_all_axis || homeY) {
        HOMEAXIS(Y);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
        #endif
      }

    #endif

    // Home X
    if (home_all_axis || homeX) {

      #if ENABLED(DUAL_X_CARRIAGE)

        // Always home the 2nd (right) extruder first
        active_extruder = 1;
        HOMEAXIS(X);

        // Remember this extruder's position for later tool change
        inactive_extruder_x_pos = RAW_X_POSITION(current_position[X_AXIS]);

        // Home the 1st (left) extruder
        active_extruder = 0;
        HOMEAXIS(X);

        // Consider the active extruder to be parked
        memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
        delayed_move_time = 0;
        active_extruder_parked = true;

      #else

        HOMEAXIS(X);

      #endif

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("> homeX", current_position);
      #endif
    }

    #if DISABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all_axis || homeY) {
        HOMEAXIS(Y);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
        #endif
      }
    #endif

    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0
      if (home_all_axis || homeZ) {
        #if ENABLED(Z_SAFE_HOMING)
          home_z_safely();
        #else
          HOMEAXIS(Z);
        #endif
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> (home_all_axis || homeZ) > final", current_position);
        #endif
      } // home_all_axis || homeZ
    #endif // Z_HOME_DIR < 0

    SYNC_PLAN_POSITION_KINEMATIC();

  #endif // !DELTA (gcode_G28)

  endstops.not_homing();

  #if ENABLED(DELTA) && ENABLED(DELTA_HOME_TO_SAFE_ZONE)
    // move to a height where we can use the full xy-area
    do_blocking_move_to_z(delta_clip_start_height);
  #endif

  // Enable mesh leveling again
  #if ENABLED(MESH_BED_LEVELING)
    if (mbl.has_mesh()) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("MBL has mesh");
      #endif
      if (home_all_axis || (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && homeZ)) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("MBL Z homing");
        #endif
        current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
          #if Z_HOME_DIR > 0
            + Z_MAX_POS
          #endif
        ;
        SYNC_PLAN_POSITION_KINEMATIC();
        mbl.set_active(true);
        #if ENABLED(MESH_G28_REST_ORIGIN)
          current_position[Z_AXIS] = 0.0;
          set_destination_to_current();
          line_to_destination(homing_feedrate_mm_s[Z_AXIS]);
          stepper.synchronize();
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("MBL Rest Origin", current_position);
          #endif
        #else
          planner.unapply_leveling(current_position);
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("MBL adjusted MESH_HOME_SEARCH_Z", current_position);
          #endif
        #endif
      }
      else if ((axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS]) && (homeX || homeY)) {
        current_position[Z_AXIS] = pre_home_z;
        SYNC_PLAN_POSITION_KINEMATIC();
        mbl.set_active(true);
        planner.unapply_leveling(current_position);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("MBL Home X or Y", current_position);
        #endif
      }
    }
  #endif

  clean_up_after_endstop_or_probe_move();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< gcode_G28");
  #endif

  // Restore the active tool after homing
  #if HOTENDS > 1
    tool_change(old_tool_index, 0, true);
  #endif

  report_current_position();
}
//&end[Homing]

#if HAS_PROBING_PROCEDURE

  void out_of_range_error(const char* p_edge) {
    SERIAL_PROTOCOLPGM("?Probe ");
    serialprintPGM(p_edge);
    SERIAL_PROTOCOLLNPGM(" position out of range.");
  }

#endif

#if ENABLED(MESH_BED_LEVELING)

  inline void _mbl_goto_xy(float x, float y) {
    float old_feedrate_mm_s = feedrate_mm_s;
    feedrate_mm_s = homing_feedrate_mm_s[Z_AXIS];

    current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
      #if Z_CLEARANCE_BETWEEN_PROBES > Z_HOMING_HEIGHT
        + Z_CLEARANCE_BETWEEN_PROBES
      #elif Z_HOMING_HEIGHT > 0
        + Z_HOMING_HEIGHT
      #endif
    ;
    line_to_current_position();

    feedrate_mm_s = MMM_TO_MMS(XY_PROBE_SPEED);
    current_position[X_AXIS] = LOGICAL_X_POSITION(x);
    current_position[Y_AXIS] = LOGICAL_Y_POSITION(y);
    line_to_current_position();

    #if Z_CLEARANCE_BETWEEN_PROBES > 0 || Z_HOMING_HEIGHT > 0
      feedrate_mm_s = homing_feedrate_mm_s[Z_AXIS];
      current_position[Z_AXIS] = LOGICAL_Z_POSITION(MESH_HOME_SEARCH_Z);
      line_to_current_position();
    #endif

    feedrate_mm_s = old_feedrate_mm_s;
    stepper.synchronize();
  }

  // Save 130 bytes with non-duplication of PSTR
  void say_not_entered() { SERIAL_PROTOCOLLNPGM(" not entered."); }

  /**
   * G29: Mesh-based Z probe, probes a grid and produces a
   *      mesh to compensate for variable bed height
   *
   * Parameters With MESH_BED_LEVELING:
   *
   *  S0              Produce a mesh report
   *  S1              Start probing mesh points
   *  S2              Probe the next mesh point
   *  S3 Xn Yn Zn.nn  Manually modify a single point
   *  S4 Zn.nn        Set z offset. Positive away from bed, negative closer to bed.
   *  S5              Reset and disable mesh
   *
   * The S0 report the points as below
   *
   *  +----> X-axis  1-n
   *  |
   *  |
   *  v Y-axis  1-n
   *
   */
  inline void gcode_G29() {

    static int probe_point = -1;
    MeshLevelingState state = code_seen('S') ? (MeshLevelingState)code_value_byte() : MeshReport;
    if (state < 0 || state > 5) {
      SERIAL_PROTOCOLLNPGM("S out of range (0-5).");
      return;
    }

    int8_t px, py;

    switch (state) {
      case MeshReport:
        if (mbl.has_mesh()) {
          SERIAL_PROTOCOLPAIR("State: ", mbl.active() ? MSG_ON : MSG_OFF);
          SERIAL_PROTOCOLLNPGM("\nNum X,Y: " STRINGIFY(MESH_NUM_X_POINTS) "," STRINGIFY(MESH_NUM_Y_POINTS));
          SERIAL_PROTOCOLLNPGM("Z search height: " STRINGIFY(MESH_HOME_SEARCH_Z));
          SERIAL_PROTOCOLPGM("Z offset: "); SERIAL_PROTOCOL_F(mbl.z_offset, 5);
          SERIAL_PROTOCOLLNPGM("\nMeasured points:");
          for (py = 0; py < MESH_NUM_Y_POINTS; py++) {
            for (px = 0; px < MESH_NUM_X_POINTS; px++) {
              SERIAL_PROTOCOLPGM("  ");
              SERIAL_PROTOCOL_F(mbl.z_values[py][px], 5);
            }
            SERIAL_EOL;
          }
        }
        else
          SERIAL_PROTOCOLLNPGM("Mesh bed leveling not active.");
        break;

      case MeshStart:
        mbl.reset();
        probe_point = 0;
        enqueue_and_echo_commands_P(PSTR("G28\nG29 S2"));
        break;

      case MeshNext:
        if (probe_point < 0) {
          SERIAL_PROTOCOLLNPGM("Start mesh probing with \"G29 S1\" first.");
          return;
        }
        // For each G29 S2...
        if (probe_point == 0) {
          // For the initial G29 S2 make Z a positive value (e.g., 4.0)
          current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
            #if Z_HOME_DIR > 0
              + Z_MAX_POS
            #endif
          ;
          SYNC_PLAN_POSITION_KINEMATIC();
        }
        else {
          // For G29 S2 after adjusting Z.
          mbl.set_zigzag_z(probe_point - 1, current_position[Z_AXIS]);
        }
        // If there's another point to sample, move there with optional lift.
        if (probe_point < (MESH_NUM_X_POINTS) * (MESH_NUM_Y_POINTS)) {
          mbl.zigzag(probe_point, px, py);
          _mbl_goto_xy(mbl.get_probe_x(px), mbl.get_probe_y(py));
          probe_point++;
        }
        else {
          // One last "return to the bed" (as originally coded) at completion
          current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
            #if Z_CLEARANCE_BETWEEN_PROBES > Z_HOMING_HEIGHT
              + Z_CLEARANCE_BETWEEN_PROBES
            #elif Z_HOMING_HEIGHT > 0
              + Z_HOMING_HEIGHT
            #endif
          ;
          line_to_current_position();
          stepper.synchronize();

          // After recording the last point, activate the mbl and home
          SERIAL_PROTOCOLLNPGM("Mesh probing done.");
          probe_point = -1;
          mbl.set_has_mesh(true);
          enqueue_and_echo_commands_P(PSTR("G28"));
        }
        break;

      case MeshSet:
        if (code_seen('X')) {
          px = code_value_int() - 1;
          if (px < 0 || px >= MESH_NUM_X_POINTS) {
            SERIAL_PROTOCOLLNPGM("X out of range (1-" STRINGIFY(MESH_NUM_X_POINTS) ").");
            return;
          }
        }
        else {
          SERIAL_CHAR('X'); say_not_entered();
          return;
        }

        if (code_seen('Y')) {
          py = code_value_int() - 1;
          if (py < 0 || py >= MESH_NUM_Y_POINTS) {
            SERIAL_PROTOCOLLNPGM("Y out of range (1-" STRINGIFY(MESH_NUM_Y_POINTS) ").");
            return;
          }
        }
        else {
          SERIAL_CHAR('Y'); say_not_entered();
          return;
        }

        if (code_seen('Z')) {
          mbl.z_values[py][px] = code_value_axis_units(Z_AXIS);
        }
        else {
          SERIAL_CHAR('Z'); say_not_entered();
          return;
        }
        break;

      case MeshSetZOffset:
        if (code_seen('Z')) {
          mbl.z_offset = code_value_axis_units(Z_AXIS);
        }
        else {
          SERIAL_CHAR('Z'); say_not_entered();
          return;
        }
        break;

      case MeshReset:
        if (mbl.active()) {
          current_position[Z_AXIS] -= MESH_HOME_SEARCH_Z;
          planner.apply_leveling(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
          mbl.reset();
          SYNC_PLAN_POSITION_KINEMATIC();
        }
        else
          mbl.reset();

    } // switch(state)

    report_current_position();
  }

#elif HAS_ABL

  /**
   * G29: Detailed Z probe, probes the bed at 3 or more points.
   *      Will fail if the printer has not been homed with G28.
   *
   * Enhanced G29 Auto Bed Leveling Probe Routine
   *
   * Parameters With ABL_GRID:
   *
   *  P  Set the size of the grid that will be probed (P x P points).
   *     Not supported by non-linear delta printer bed leveling.
   *     Example: "G29 P4"
   *
   *  S  Set the XY travel speed between probe points (in units/min)
   *
   *  D  Dry-Run mode. Just evaluate the bed Topology - Don't apply
   *     or clean the rotation Matrix. Useful to check the topology
   *     after a first run of G29.
   *
   *  V  Set the verbose level (0-4). Example: "G29 V3"
   *
   *  T  Generate a Bed Topology Report. Example: "G29 P5 T" for a detailed report.
   *     This is useful for manual bed leveling and finding flaws in the bed (to
   *     assist with part placement).
   *     Not supported by non-linear delta printer bed leveling.
   *
   *  F  Set the Front limit of the probing grid
   *  B  Set the Back limit of the probing grid
   *  L  Set the Left limit of the probing grid
   *  R  Set the Right limit of the probing grid
   *
   * Global Parameters:
   *
   * E/e By default G29 will engage the Z probe, test the bed, then disengage.
   *     Include "E" to engage/disengage the Z probe for each sample.
   *     There's no extra effect if you have a fixed Z probe.
   *     Usage: "G29 E" or "G29 e"
   *
   */
   
  inline void gcode_G29() {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      bool query = code_seen('Q');
      uint8_t old_debug_flags = marlin_debug_flags;
      if (query) marlin_debug_flags |= DEBUG_LEVELING;
      if (DEBUGGING(LEVELING)) {
        DEBUG_POS(">>> gcode_G29", current_position);
        log_machine_info();
      }
      marlin_debug_flags = old_debug_flags;
      if (query) return;
    #endif

    // Don't allow auto-leveling without homing first
    if (axis_unhomed_error(true, true, true)) return;

    int verbose_level = code_seen('V') ? code_value_int() : 1;
    if (verbose_level < 0 || verbose_level > 4) {
      SERIAL_PROTOCOLLNPGM("?(V)erbose Level is implausible (0-4).");
      return;
    }

    bool dryrun = code_seen('D'),
         stow_probe_after_each = code_seen('E');

    #if ABL_GRID

      if (verbose_level > 0) {
        SERIAL_PROTOCOLLNPGM("G29 Auto Bed Leveling");
        if (dryrun) SERIAL_PROTOCOLLNPGM("Running in DRY-RUN mode");
      }

      #if ABL_PLANAR

        bool do_topography_map = verbose_level > 2 || code_seen('T');

        // X and Y specify points in each direction, overriding the default
        // These values may be saved with the completed mesh
        int abl_grid_points_x = code_seen('X') ? code_value_int() : ABL_GRID_POINTS_X,
            abl_grid_points_y = code_seen('Y') ? code_value_int() : ABL_GRID_POINTS_Y;

        if (code_seen('P')) abl_grid_points_x = abl_grid_points_y = code_value_int();

        if (abl_grid_points_x < 2 || abl_grid_points_y < 2) {
          SERIAL_PROTOCOLLNPGM("?Number of probe points is implausible (2 minimum).");
          return;
        }

      #else

         const int abl_grid_points_x = ABL_GRID_POINTS_X, abl_grid_points_y = ABL_GRID_POINTS_Y;

      #endif

      xy_probe_feedrate_mm_s = MMM_TO_MMS(code_seen('S') ? code_value_linear_units() : XY_PROBE_SPEED);

      int left_probe_bed_position = code_seen('L') ? (int)code_value_axis_units(X_AXIS) : LOGICAL_X_POSITION(LEFT_PROBE_BED_POSITION),
          right_probe_bed_position = code_seen('R') ? (int)code_value_axis_units(X_AXIS) : LOGICAL_X_POSITION(RIGHT_PROBE_BED_POSITION),
          front_probe_bed_position = code_seen('F') ? (int)code_value_axis_units(Y_AXIS) : LOGICAL_Y_POSITION(FRONT_PROBE_BED_POSITION),
          back_probe_bed_position = code_seen('B') ? (int)code_value_axis_units(Y_AXIS) : LOGICAL_Y_POSITION(BACK_PROBE_BED_POSITION);

      bool left_out_l = left_probe_bed_position < LOGICAL_X_POSITION(MIN_PROBE_X),
           left_out = left_out_l || left_probe_bed_position > right_probe_bed_position - (MIN_PROBE_EDGE),
           right_out_r = right_probe_bed_position > LOGICAL_X_POSITION(MAX_PROBE_X),
           right_out = right_out_r || right_probe_bed_position < left_probe_bed_position + MIN_PROBE_EDGE,
           front_out_f = front_probe_bed_position < LOGICAL_Y_POSITION(MIN_PROBE_Y),
           front_out = front_out_f || front_probe_bed_position > back_probe_bed_position - (MIN_PROBE_EDGE),
           back_out_b = back_probe_bed_position > LOGICAL_Y_POSITION(MAX_PROBE_Y),
           back_out = back_out_b || back_probe_bed_position < front_probe_bed_position + MIN_PROBE_EDGE;

      if (left_out || right_out || front_out || back_out) {
        if (left_out) {
          out_of_range_error(PSTR("(L)eft"));
          left_probe_bed_position = left_out_l ? LOGICAL_X_POSITION(MIN_PROBE_X) : right_probe_bed_position - (MIN_PROBE_EDGE);
        }
        if (right_out) {
          out_of_range_error(PSTR("(R)ight"));
          right_probe_bed_position = right_out_r ? LOGICAL_Y_POSITION(MAX_PROBE_X) : left_probe_bed_position + MIN_PROBE_EDGE;
        }
        if (front_out) {
          out_of_range_error(PSTR("(F)ront"));
          front_probe_bed_position = front_out_f ? LOGICAL_Y_POSITION(MIN_PROBE_Y) : back_probe_bed_position - (MIN_PROBE_EDGE);
        }
        if (back_out) {
          out_of_range_error(PSTR("(B)ack"));
          back_probe_bed_position = back_out_b ? LOGICAL_Y_POSITION(MAX_PROBE_Y) : front_probe_bed_position + MIN_PROBE_EDGE;
        }
        return;
      }

    #endif // ABL_GRID

    stepper.synchronize();

    // Disable auto bed leveling during G29
    bool abl_should_enable = planner.abl_enabled;

    planner.abl_enabled = false;

    if (!dryrun) {
      // Re-orient the current position without leveling
      // based on where the steppers are positioned.
      set_current_from_steppers_for_axis(ALL_AXES);

      // Sync the planner to where the steppers stopped
      SYNC_PLAN_POSITION_KINEMATIC();
    }

    setup_for_endstop_or_probe_move();

    // Deploy the probe. Probe will raise if needed.
    if (DEPLOY_PROBE()) {
      planner.abl_enabled = abl_should_enable;
      return;
    }

    float xProbe = 0, yProbe = 0, measured_z = 0;

    #if ABL_GRID

      // probe at the points of a lattice grid
      const float xGridSpacing = (right_probe_bed_position - left_probe_bed_position) / (abl_grid_points_x - 1),
                  yGridSpacing = (back_probe_bed_position - front_probe_bed_position) / (abl_grid_points_y - 1);

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

        float zoffset = zprobe_zoffset;
        if (code_seen('Z')) zoffset += code_value_axis_units(Z_AXIS);

        if ( xGridSpacing != bilinear_grid_spacing[X_AXIS]
          || yGridSpacing != bilinear_grid_spacing[Y_AXIS]
          || left_probe_bed_position != bilinear_start[X_AXIS]
          || front_probe_bed_position != bilinear_start[Y_AXIS]
        ) {
          reset_bed_level();
          #if ENABLED(ABL_BILINEAR_SUBDIVISION)
            bilinear_grid_spacing_virt[X_AXIS] = xGridSpacing / (BILINEAR_SUBDIVISIONS);
            bilinear_grid_spacing_virt[Y_AXIS] = yGridSpacing / (BILINEAR_SUBDIVISIONS);
          #endif
          bilinear_grid_spacing[X_AXIS] = xGridSpacing;
          bilinear_grid_spacing[Y_AXIS] = yGridSpacing;
          bilinear_start[X_AXIS] = RAW_X_POSITION(left_probe_bed_position);
          bilinear_start[Y_AXIS] = RAW_Y_POSITION(front_probe_bed_position);
          // Can't re-enable (on error) until the new grid is written
          abl_should_enable = false;
        }

      #elif ENABLED(AUTO_BED_LEVELING_LINEAR)

        /**
         * solve the plane equation ax + by + d = z
         * A is the matrix with rows [x y 1] for all the probed points
         * B is the vector of the Z positions
         * the normal vector to the plane is formed by the coefficients of the
         * plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
         * so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z
         */

        int abl2 = abl_grid_points_x * abl_grid_points_y,
            indexIntoAB[abl_grid_points_x][abl_grid_points_y],
            probePointCounter = -1;

        float eqnAMatrix[abl2 * 3], // "A" matrix of the linear system of equations
              eqnBVector[abl2],     // "B" vector of Z points
              mean = 0.0;

      #endif // AUTO_BED_LEVELING_LINEAR

      #if ENABLED(PROBE_Y_FIRST)
        #define PR_OUTER_VAR xCount
        #define PR_OUTER_END abl_grid_points_x
        #define PR_INNER_VAR yCount
        #define PR_INNER_END abl_grid_points_y
      #else
        #define PR_OUTER_VAR yCount
        #define PR_OUTER_END abl_grid_points_y
        #define PR_INNER_VAR xCount
        #define PR_INNER_END abl_grid_points_x
      #endif

      bool zig = PR_OUTER_END & 1;  // Always end at RIGHT and BACK_PROBE_BED_POSITION

      // Outer loop is Y with PROBE_Y_FIRST disabled
      for (uint8_t PR_OUTER_VAR = 0; PR_OUTER_VAR < PR_OUTER_END; PR_OUTER_VAR++) {

        int8_t inStart, inStop, inInc;

        if (zig) { // away from origin
          inStart = 0;
          inStop = PR_INNER_END;
          inInc = 1;
        }
        else {     // towards origin
          inStart = PR_INNER_END - 1;
          inStop = -1;
          inInc = -1;
        }

        zig = !zig; // zag

        // Inner loop is Y with PROBE_Y_FIRST enabled
        for (int8_t PR_INNER_VAR = inStart; PR_INNER_VAR != inStop; PR_INNER_VAR += inInc) {

          float xBase = left_probe_bed_position + xGridSpacing * xCount,
                yBase = front_probe_bed_position + yGridSpacing * yCount;

          xProbe = floor(xBase + (xBase < 0 ? 0 : 0.5));
          yProbe = floor(yBase + (yBase < 0 ? 0 : 0.5));

          #if ENABLED(AUTO_BED_LEVELING_LINEAR)
            indexIntoAB[xCount][yCount] = ++probePointCounter;
          #endif

          #if IS_KINEMATIC
            // Avoid probing outside the round or hexagonal area
            float pos[XYZ] = { xProbe, yProbe, 0 };
            if (!position_is_reachable(pos, true)) continue;
          #endif

          measured_z = probe_pt(xProbe, yProbe, stow_probe_after_each, verbose_level);

          if (measured_z == NAN) {
            planner.abl_enabled = abl_should_enable;
            return;
          }

          #if ENABLED(AUTO_BED_LEVELING_LINEAR)

            mean += measured_z;
            eqnBVector[probePointCounter] = measured_z;
            eqnAMatrix[probePointCounter + 0 * abl2] = xProbe;
            eqnAMatrix[probePointCounter + 1 * abl2] = yProbe;
            eqnAMatrix[probePointCounter + 2 * abl2] = 1;

          #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

            bed_level_grid[xCount][yCount] = measured_z + zoffset;

          #endif

          idle();

        } //xProbe
      } //yProbe

    #elif ENABLED(AUTO_BED_LEVELING_3POINT)

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("> 3-point Leveling");
      #endif

      // Probe at 3 arbitrary points
      vector_3 points[3] = {
        vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, 0),
        vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, 0),
        vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, 0)
      };

      for (uint8_t i = 0; i < 3; ++i) {
        // Retain the last probe position
        xProbe = LOGICAL_X_POSITION(points[i].x);
        yProbe = LOGICAL_Y_POSITION(points[i].y);
        measured_z = points[i].z = probe_pt(xProbe, yProbe, stow_probe_after_each, verbose_level);
      }

      if (measured_z == NAN) {
        planner.abl_enabled = abl_should_enable;
        return;
      }

      if (!dryrun) {
        vector_3 planeNormal = vector_3::cross(points[0] - points[1], points[2] - points[1]).get_normal();
        if (planeNormal.z < 0) {
          planeNormal.x *= -1;
          planeNormal.y *= -1;
          planeNormal.z *= -1;
        }
        planner.bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

        // Can't re-enable (on error) until the new grid is written
        abl_should_enable = false;
      }

    #endif // AUTO_BED_LEVELING_3POINT

    // Raise to _Z_CLEARANCE_DEPLOY_PROBE. Stow the probe.
    if (STOW_PROBE()) {
      planner.abl_enabled = abl_should_enable;
      return;
    }

    //
    // Unless this is a dry run, auto bed leveling will
    // definitely be enabled after this point
    //

    // Restore state after probing
    clean_up_after_endstop_or_probe_move();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("> probing complete", current_position);
    #endif

    // Calculate leveling, print reports, correct the position
    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

      if (!dryrun) extrapolate_unprobed_bed_level();
      print_bed_level();

      #if ENABLED(ABL_BILINEAR_SUBDIVISION)
        bed_level_virt_prepare();
        bed_level_virt_interpolate();
        bed_level_virt_print();
      #endif

    #elif ENABLED(AUTO_BED_LEVELING_LINEAR)

      // For LINEAR leveling calculate matrix, print reports, correct the position

      // solve lsq problem
      float plane_equation_coefficients[3];
      qr_solve(plane_equation_coefficients, abl2, 3, eqnAMatrix, eqnBVector);

      mean /= abl2;

      if (verbose_level) {
        SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
        SERIAL_PROTOCOL_F(plane_equation_coefficients[0], 8);
        SERIAL_PROTOCOLPGM(" b: ");
        SERIAL_PROTOCOL_F(plane_equation_coefficients[1], 8);
        SERIAL_PROTOCOLPGM(" d: ");
        SERIAL_PROTOCOL_F(plane_equation_coefficients[2], 8);
        SERIAL_EOL;
        if (verbose_level > 2) {
          SERIAL_PROTOCOLPGM("Mean of sampled points: ");
          SERIAL_PROTOCOL_F(mean, 8);
          SERIAL_EOL;
        }
      }

      // Create the matrix but don't correct the position yet
      if (!dryrun) {
        planner.bed_level_matrix = matrix_3x3::create_look_at(
          vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1)
        );
      }

      // Show the Topography map if enabled
      if (do_topography_map) {

        SERIAL_PROTOCOLLNPGM("\nBed Height Topography:\n"
                               "   +--- BACK --+\n"
                               "   |           |\n"
                               " L |    (+)    | R\n"
                               " E |           | I\n"
                               " F | (-) N (+) | G\n"
                               " T |           | H\n"
                               "   |    (-)    | T\n"
                               "   |           |\n"
                               "   O-- FRONT --+\n"
                               " (0,0)");

        float min_diff = 999;

        for (int8_t yy = abl_grid_points_y - 1; yy >= 0; yy--) {
          for (uint8_t xx = 0; xx < abl_grid_points_x; xx++) {
            int ind = indexIntoAB[xx][yy];
            float diff = eqnBVector[ind] - mean,
                  x_tmp = eqnAMatrix[ind + 0 * abl2],
                  y_tmp = eqnAMatrix[ind + 1 * abl2],
                  z_tmp = 0;

            apply_rotation_xyz(planner.bed_level_matrix, x_tmp, y_tmp, z_tmp);

            NOMORE(min_diff, eqnBVector[ind] - z_tmp);

            if (diff >= 0.0)
              SERIAL_PROTOCOLPGM(" +");   // Include + for column alignment
            else
              SERIAL_PROTOCOLCHAR(' ');
            SERIAL_PROTOCOL_F(diff, 5);
          } // xx
          SERIAL_EOL;
        } // yy
        SERIAL_EOL;

        if (verbose_level > 3) {
          SERIAL_PROTOCOLLNPGM("\nCorrected Bed Height vs. Bed Topology:");

          for (int8_t yy = abl_grid_points_y - 1; yy >= 0; yy--) {
            for (uint8_t xx = 0; xx < abl_grid_points_x; xx++) {
              int ind = indexIntoAB[xx][yy];
              float x_tmp = eqnAMatrix[ind + 0 * abl2],
                    y_tmp = eqnAMatrix[ind + 1 * abl2],
                    z_tmp = 0;

              apply_rotation_xyz(planner.bed_level_matrix, x_tmp, y_tmp, z_tmp);

              float diff = eqnBVector[ind] - z_tmp - min_diff;
              if (diff >= 0.0)
                SERIAL_PROTOCOLPGM(" +");
              // Include + for column alignment
              else
                SERIAL_PROTOCOLCHAR(' ');
              SERIAL_PROTOCOL_F(diff, 5);
            } // xx
            SERIAL_EOL;
          } // yy
          SERIAL_EOL;
        }
      } //do_topography_map

    #endif // AUTO_BED_LEVELING_LINEAR

    #if ABL_PLANAR

      // For LINEAR and 3POINT leveling correct the current position

      if (verbose_level > 0)
        planner.bed_level_matrix.debug("\n\nBed Level Correction Matrix:");

      if (!dryrun) {
        //
        // Correct the current XYZ position based on the tilted plane.
        //

        // 1. Get the distance from the current position to the reference point.
        float x_dist = RAW_CURRENT_POSITION(X_AXIS) - X_TILT_FULCRUM,
              y_dist = RAW_CURRENT_POSITION(Y_AXIS) - Y_TILT_FULCRUM,
              z_real = current_position[Z_AXIS],
              z_zero = 0;

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("G29 uncorrected XYZ", current_position);
        #endif
        
		matrix_3x3 inverse = matrix_3x3::transpose(planner.bed_level_matrix);

        // 2. Apply the inverse matrix to the distance
        //    from the reference point to X, Y, and zero.
        apply_rotation_xyz(inverse, x_dist, y_dist, z_zero);

        // 3. Get the matrix-based corrected Z.
        //    (Even if not used, get it for comparison.)
        float new_z = z_real + z_zero;

        // 4. Use the last measured distance to the bed, if possible
        if ( NEAR(current_position[X_AXIS], xProbe - (X_PROBE_OFFSET_FROM_EXTRUDER))
          && NEAR(current_position[Y_AXIS], yProbe - (Y_PROBE_OFFSET_FROM_EXTRUDER))
        ) {
          float simple_z = z_real - (measured_z - (-zprobe_zoffset));
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOPAIR("Z from Probe:", simple_z);
              SERIAL_ECHOPAIR("  Matrix:", new_z);
              SERIAL_ECHOLNPAIR("  Discrepancy:", simple_z - new_z);
            }
          #endif
          new_z = simple_z;
        }

        // 5. The rotated XY and corrected Z are now current_position
        current_position[X_AXIS] = LOGICAL_X_POSITION(x_dist) + X_TILT_FULCRUM;
        current_position[Y_AXIS] = LOGICAL_Y_POSITION(y_dist) + Y_TILT_FULCRUM;
        current_position[Z_AXIS] = new_z;

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("G29 corrected XYZ", current_position);
        #endif
      }

    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

      if (!dryrun) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR("G29 uncorrected Z:", current_position[Z_AXIS]);
        #endif

        // Unapply the offset because it is going to be immediately applied
        // and cause compensation movement in Z
        current_position[Z_AXIS] -= bilinear_z_offset(current_position);

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR(" corrected Z:", current_position[Z_AXIS]);
        #endif
      }

    #endif // ABL_PLANAR

    #ifdef Z_PROBE_END_SCRIPT
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR("Z Probe End Script: ", Z_PROBE_END_SCRIPT);
      #endif
      enqueue_and_echo_commands_P(PSTR(Z_PROBE_END_SCRIPT));
      stepper.synchronize();
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< gcode_G29");
    #endif

    report_current_position();

    KEEPALIVE_STATE(IN_HANDLER);

    // Auto Bed Leveling is complete! Enable if possible.
    planner.abl_enabled = dryrun ? abl_should_enable : true;

    if (planner.abl_enabled)
      SYNC_PLAN_POSITION_KINEMATIC();
  }

#endif // HAS_ABL

#if HAS_BED_PROBE

  /**
   * G30: Do a single Z probe at the current XY
   * Usage:
   *   G30 <X#> <Y#> <S#>
   *     X = Probe X position (default=current probe position)
   *     Y = Probe Y position (default=current probe position)
   *     S = Stows the probe if 1 (default=1)
   */
  inline void gcode_G30() {
    float X_probe_location = code_seen('X') ? code_value_axis_units(X_AXIS) : current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER,
          Y_probe_location = code_seen('Y') ? code_value_axis_units(Y_AXIS) : current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER;

    float pos[XYZ] = { X_probe_location, Y_probe_location, LOGICAL_Z_POSITION(0) };
    if (!position_is_reachable(pos, true)) return;

    bool stow = code_seen('S') ? code_value_bool() : true;

    // Disable leveling so the planner won't mess with us
    #if PLANNER_LEVELING
      set_bed_leveling_enabled(false);
    #endif

    setup_for_endstop_or_probe_move();

    float measured_z = probe_pt(X_probe_location, Y_probe_location, stow, 1);

    SERIAL_PROTOCOLPGM("Bed X: ");
    SERIAL_PROTOCOL(X_probe_location + 0.0001);
    SERIAL_PROTOCOLPGM(" Y: ");
    SERIAL_PROTOCOL(Y_probe_location + 0.0001);
    SERIAL_PROTOCOLPGM(" Z: ");
    SERIAL_PROTOCOLLN(measured_z + 0.0001);

    clean_up_after_endstop_or_probe_move();

    report_current_position();
  }

  #if ENABLED(Z_PROBE_SLED)

    /**
     * G31: Deploy the Z probe
     */
    inline void gcode_G31() { DEPLOY_PROBE(); }

    /**
     * G32: Stow the Z probe
     */
    inline void gcode_G32() { STOW_PROBE(); }

  #endif // Z_PROBE_SLED

#endif // HAS_BED_PROBE

#if ENABLED(G38_PROBE_TARGET)

  static bool G38_run_probe() {

    bool G38_pass_fail = false;

    // Get direction of move and retract
    float retract_mm[XYZ];
    LOOP_XYZ(i) {
      float dist = destination[i] - current_position[i];
      retract_mm[i] = fabs(dist) < G38_MINIMUM_MOVE ? 0 : home_bump_mm((AxisEnum)i) * (dist > 0 ? -1 : 1);
    }

    stepper.synchronize();  // wait until the machine is idle

    // Move until destination reached or target hit
    endstops.enable(true);
    G38_move = true;
    G38_endstop_hit = false;
    prepare_move_to_destination();
    stepper.synchronize();
    G38_move = false;

    endstops.hit_on_purpose();
    set_current_from_steppers_for_axis(ALL_AXES);
    SYNC_PLAN_POSITION_KINEMATIC();

    // Only do remaining moves if target was hit
    if (G38_endstop_hit) {

      G38_pass_fail = true;

      // Move away by the retract distance
      set_destination_to_current();
      LOOP_XYZ(i) destination[i] += retract_mm[i];
      endstops.enable(false);
      prepare_move_to_destination();
      stepper.synchronize();

      feedrate_mm_s /= 4;

      // Bump the target more slowly
      LOOP_XYZ(i) destination[i] -= retract_mm[i] * 2;

      endstops.enable(true);
      G38_move = true;
      prepare_move_to_destination();
      stepper.synchronize();
      G38_move = false;

      set_current_from_steppers_for_axis(ALL_AXES);
      SYNC_PLAN_POSITION_KINEMATIC();
    }

    endstops.hit_on_purpose();
    endstops.not_homing();
    return G38_pass_fail;
  }

  /**
   * G38.2 - probe toward workpiece, stop on contact, signal error if failure
   * G38.3 - probe toward workpiece, stop on contact
   *
   * Like G28 except uses Z min endstop for all axes
   */
  inline void gcode_G38(bool is_38_2) {
    // Get X Y Z E F
    gcode_get_destination();

    setup_for_endstop_or_probe_move();

    // If any axis has enough movement, do the move
    LOOP_XYZ(i)
      if (fabs(destination[i] - current_position[i]) >= G38_MINIMUM_MOVE) {
        if (!code_seen('F')) feedrate_mm_s = homing_feedrate_mm_s[i];
        // If G38.2 fails throw an error
        if (!G38_run_probe() && is_38_2) {
          SERIAL_ERROR_START;
          SERIAL_ERRORLNPGM("Failed to reach target");
        }
        break;
      }

    clean_up_after_endstop_or_probe_move();
  }

#endif // G38_PROBE_TARGET

//&begin[Move_To_Destination]
/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {
  bool didXYZ = false,
       didE = code_seen('E');

  if (!didE) stepper.synchronize();

  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i])) {
      #if IS_SCARA
        current_position[i] = code_value_axis_units(i);
        if (i != E_AXIS) didXYZ = true;
      #else
        float p = current_position[i],
              v = code_value_axis_units(i);

        current_position[i] = v;

        if (i != E_AXIS) {
          didXYZ = true;
          position_shift[i] += v - p; // Offset the coordinate space
          update_software_endstops((AxisEnum)i);
        }
      #endif
    }
  }
  if (didXYZ)
    SYNC_PLAN_POSITION_KINEMATIC();
  else if (didE)
    sync_plan_position_e();

  report_current_position();
}
//&end[Move_To_Destination]

#if ENABLED(EMERGENCY_PARSER) || ENABLED(ULTIPANEL)

  /**
   * M0: Unconditional stop - Wait for user button press on LCD
   * M1: Conditional stop   - Wait for user button press on LCD
   */
  inline void gcode_M0_M1() {
    char* args = current_command_args;

    millis_t codenum = 0;
    bool hasP = false, hasS = false;
    if (code_seen('P')) {
      codenum = code_value_millis(); // milliseconds to wait
      hasP = codenum > 0;
    }
    if (code_seen('S')) {
      codenum = code_value_millis_from_seconds(); // seconds to wait
      hasS = codenum > 0;
    }

    #if ENABLED(ULTIPANEL)

      if (!hasP && !hasS && *args != '\0')
        lcd_setstatus(args, true);
      else {
        LCD_MESSAGEPGM(MSG_USERWAIT);
        #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
          dontExpireStatus();
        #endif
      }

    #else

      if (!hasP && !hasS && *args != '\0') {
        SERIAL_ECHO_START;
        SERIAL_ECHOLN(args);
      }

    #endif

    wait_for_user = true;
    KEEPALIVE_STATE(PAUSED_FOR_USER);

    stepper.synchronize();
    refresh_cmd_timeout();

    if (codenum > 0) {
      codenum += previous_cmd_ms;  // wait until this time for a click
      while (PENDING(millis(), codenum) && wait_for_user) idle();
    }
    else {
      #if ENABLED(ULTIPANEL)
        if (lcd_detected()) {
          while (wait_for_user) idle();
          IS_SD_PRINTING ? LCD_MESSAGEPGM(MSG_RESUMING) : LCD_MESSAGEPGM(WELCOME_MSG);
        }
      #else
        while (wait_for_user) idle();
      #endif
    }

    wait_for_user = false;
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif // EMERGENCY_PARSER || ULTIPANEL

//&begin[Moter_Type_Stepper]
/**
 * M17: Enable power on all stepper motors
 */
inline void gcode_M17() {
  LCD_MESSAGEPGM(MSG_NO_MOVE);
  enable_all_steppers();
}
//&end[Moter_Type_Stepper]

#if ENABLED(SDSUPPORT)

  /**
   * M20: List SD card to serial output
   */
  inline void gcode_M20() {
    SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
    card.ls();
    SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
  }

  /**
   * M21: Init SD Card
   */
  inline void gcode_M21() { card.initsd(); }

  /**
   * M22: Release SD Card
   */
  inline void gcode_M22() { card.release(); }

  /**
   * M23: Open a file
   */
  inline void gcode_M23() { card.openFile(current_command_args, true); }

  /**
   * M24: Start SD Print
   */
  inline void gcode_M24() {
    card.startFileprint();
    print_job_timer.start();
  }

  /**
   * M25: Pause SD Print
   */
  inline void gcode_M25() { card.pauseSDPrint(); }

  /**
   * M26: Set SD Card file index
   */
  inline void gcode_M26() {
    if (card.cardOK && code_seen('S'))
      card.setIndex(code_value_long());
  }

  /**
   * M27: Get SD Card status
   */
  inline void gcode_M27() { card.getStatus(); }

  /**
   * M28: Start SD Write
   */
  inline void gcode_M28() { card.openFile(current_command_args, false); }

  /**
   * M29: Stop SD Write
   * Processed in write to file routine above
   */
  inline void gcode_M29() {
    // card.saving = false;
  }

  /**
   * M30 <filename>: Delete SD Card file
   */
  inline void gcode_M30() {
    if (card.cardOK) {
      card.closefile();
      card.removeFile(current_command_args);
    }
  }

#endif // SDSUPPORT

/**
 * M31: Get the time since the start of SD Print (or last M109)
 */
inline void gcode_M31() {
  char buffer[21];
  duration_t elapsed = print_job_timer.duration();
  elapsed.toString(buffer);

  lcd_setstatus(buffer);

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPAIR("Print time: ", buffer);

  #if ENABLED(AUTOTEMP)
    thermalManager.autotempShutdown();
  #endif
}

#if ENABLED(SDSUPPORT)

  /**
   * M32: Select file and start SD Print
   */
  inline void gcode_M32() {
    if (card.sdprinting)
      stepper.synchronize();

    char* namestartpos = strchr(current_command_args, '!');  // Find ! to indicate filename string start.
    if (!namestartpos)
      namestartpos = current_command_args; // Default name position, 4 letters after the M
    else
      namestartpos++; //to skip the '!'

    bool call_procedure = code_seen('P') && (seen_pointer < namestartpos);

    if (card.cardOK) {
      card.openFile(namestartpos, true, call_procedure);

      if (code_seen('S') && seen_pointer < namestartpos) // "S" (must occur _before_ the filename!)
        card.setIndex(code_value_long());

      card.startFileprint();

      // Procedure calls count as normal print time.
      if (!call_procedure) print_job_timer.start();
    }
  }

  #if ENABLED(LONG_FILENAME_HOST_SUPPORT)

    /**
     * M33: Get the long full path of a file or folder
     *
     * Parameters:
     *   <dospath> Case-insensitive DOS-style path to a file or folder
     *
     * Example:
     *   M33 miscel~1/armchair/armcha~1.gco
     *
     * Output:
     *   /Miscellaneous/Armchair/Armchair.gcode
     */
    inline void gcode_M33() {
      card.printLongPath(current_command_args);
    }

  #endif

  /**
   * M928: Start SD Write
   */
  inline void gcode_M928() {
    card.openLogFile(current_command_args);
  }

#endif // SDSUPPORT

//&begin[Board]
/**
 * Sensitive pin test for M42, M226
 */
static bool pin_is_protected(uint8_t pin) {
  static const int sensitive_pins[] = SENSITIVE_PINS;
  for (uint8_t i = 0; i < COUNT(sensitive_pins); i++)
    if (sensitive_pins[i] == pin) return true;
  return false;
}

/**
 * M42: Change pin status via GCode
 *
 *  P<pin>  Pin number (LED if omitted)
 *  S<byte> Pin status from 0 - 255
 */
inline void gcode_M42() {
  if (!code_seen('S')) return;

  int pin_status = code_value_int();
  if (pin_status < 0 || pin_status > 255) return;

  int pin_number = code_seen('P') ? code_value_int() : LED_PIN;
  if (pin_number < 0) return;

  if (pin_is_protected(pin_number)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_PROTECTED_PIN);
    return;
  }

  pinMode(pin_number, OUTPUT);
  digitalWrite(pin_number, pin_status);
  analogWrite(pin_number, pin_status);

  #if FAN_COUNT > 0
    switch (pin_number) {
      #if HAS_FAN0
        case FAN_PIN: fanSpeeds[0] = pin_status; break;
      #endif
      #if HAS_FAN1
        case FAN1_PIN: fanSpeeds[1] = pin_status; break;
      #endif
      #if HAS_FAN2
        case FAN2_PIN: fanSpeeds[2] = pin_status; break;
      #endif
    }
  #endif
}
//&end[Board]

#if ENABLED(PINS_DEBUGGING)

  #include "pinsDebug.h"

  /**
   * M43: Pin report and debug
   *
   *      E<bool> Enable / disable background endstop monitoring
   *               - Machine continues to operate
   *               - Reports changes to endstops
   *               - Toggles LED when an endstop changes
   *
   *   or
   *
   *      P<pin>  Pin to read or watch. If omitted, read/watch all pins.
   *      W<bool> Watch pins -reporting changes- until reset, click, or M108.
   *      I<bool> Flag to ignore Marlin's pin protection.
   *
   */
  inline void gcode_M43() {

    // Enable or disable endstop monitoring
    if (code_seen('E')) {
      endstop_monitor_flag = code_value_bool();
      SERIAL_PROTOCOLPGM("endstop monitor ");
      SERIAL_PROTOCOL(endstop_monitor_flag ? "en" : "dis");
      SERIAL_PROTOCOLLNPGM("abled");
      return;
    }

    // Get the range of pins to test or watch
    int first_pin = 0, last_pin = NUM_DIGITAL_PINS - 1;
    if (code_seen('P')) {
      first_pin = last_pin = code_value_byte();
      if (first_pin > NUM_DIGITAL_PINS - 1) return;
    }

    bool ignore_protection = code_seen('I') ? code_value_bool() : false;

    // Watch until click, M108, or reset
    if (code_seen('W') && code_value_bool()) { // watch digital pins
      byte pin_state[last_pin - first_pin + 1];
      for (int8_t pin = first_pin; pin <= last_pin; pin++) {
        if (pin_is_protected(pin) && !ignore_protection) continue;
        pinMode(pin, INPUT_PULLUP);
        // if (IS_ANALOG(pin))
        //   pin_state[pin - first_pin] = analogRead(pin - analogInputToDigitalPin(0)); // int16_t pin_state[...]
        // else
          pin_state[pin - first_pin] = digitalRead(pin);
      }

      #if ENABLED(EMERGENCY_PARSER) || ENABLED(ULTIPANEL)
        wait_for_user = true;
      #endif

      for(;;) {
        for (int8_t pin = first_pin; pin <= last_pin; pin++) {
          if (pin_is_protected(pin)) continue;
          byte val;
          // if (IS_ANALOG(pin))
          //   val = analogRead(pin - analogInputToDigitalPin(0)); // int16_t val
          // else
            val = digitalRead(pin);
          if (val != pin_state[pin - first_pin]) {
            report_pin_state(pin);
            pin_state[pin - first_pin] = val;
          }
        }

        #if ENABLED(EMERGENCY_PARSER) || ENABLED(ULTIPANEL)
          if (!wait_for_user) break;
        #endif

        safe_delay(500);
      }
      return;
    }

    // Report current state of selected pin(s)
    for (uint8_t pin = first_pin; pin <= last_pin; pin++)
      report_pin_state_extended(pin, ignore_protection);
  }

#endif // PINS_DEBUGGING

#if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)

  /**
   * M48: Z probe repeatability measurement function.
   *
   * Usage:
   *   M48 <P#> <X#> <Y#> <V#> <E> <L#>
   *     P = Number of sampled points (4-50, default 10)
   *     X = Sample X position
   *     Y = Sample Y position
   *     V = Verbose level (0-4, default=1)
   *     E = Engage Z probe for each reading
   *     L = Number of legs of movement before probe
   *     S = Schizoid (Or Star if you prefer)
   *
   * This function assumes the bed has been homed.  Specifically, that a G28 command
   * as been issued prior to invoking the M48 Z probe repeatability measurement function.
   * Any information generated by a prior G29 Bed leveling command will be lost and need to be
   * regenerated.
   */
  inline void gcode_M48() {

    if (axis_unhomed_error(true, true, true)) return;

    int8_t verbose_level = code_seen('V') ? code_value_byte() : 1;
    if (verbose_level < 0 || verbose_level > 4) {
      SERIAL_PROTOCOLLNPGM("?Verbose Level not plausible (0-4).");
      return;
    }

    if (verbose_level > 0)
      SERIAL_PROTOCOLLNPGM("M48 Z-Probe Repeatability Test");

    int8_t n_samples = code_seen('P') ? code_value_byte() : 10;
    if (n_samples < 4 || n_samples > 50) {
      SERIAL_PROTOCOLLNPGM("?Sample size not plausible (4-50).");
      return;
    }

    float  X_current = current_position[X_AXIS],
           Y_current = current_position[Y_AXIS];

    bool stow_probe_after_each = code_seen('E');

    float X_probe_location = code_seen('X') ? code_value_axis_units(X_AXIS) : X_current + X_PROBE_OFFSET_FROM_EXTRUDER;
    #if DISABLED(DELTA)
      if (X_probe_location < LOGICAL_X_POSITION(MIN_PROBE_X) || X_probe_location > LOGICAL_X_POSITION(MAX_PROBE_X)) {
        out_of_range_error(PSTR("X"));
        return;
      }
    #endif

    float Y_probe_location = code_seen('Y') ? code_value_axis_units(Y_AXIS) : Y_current + Y_PROBE_OFFSET_FROM_EXTRUDER;
    #if DISABLED(DELTA)
      if (Y_probe_location < LOGICAL_Y_POSITION(MIN_PROBE_Y) || Y_probe_location > LOGICAL_Y_POSITION(MAX_PROBE_Y)) {
        out_of_range_error(PSTR("Y"));
        return;
      }
    #else
      float pos[XYZ] = { X_probe_location, Y_probe_location, 0 };
      if (!position_is_reachable(pos, true)) {
        SERIAL_PROTOCOLLNPGM("? (X,Y) location outside of probeable radius.");
        return;
      }
    #endif

    bool seen_L = code_seen('L');
    uint8_t n_legs = seen_L ? code_value_byte() : 0;
    if (n_legs > 15) {
      SERIAL_PROTOCOLLNPGM("?Number of legs in movement not plausible (0-15).");
      return;
    }
    if (n_legs == 1) n_legs = 2;

    bool schizoid_flag = code_seen('S');
    if (schizoid_flag && !seen_L) n_legs = 7;

    /**
     * Now get everything to the specified probe point So we can safely do a
     * probe to get us close to the bed.  If the Z-Axis is far from the bed,
     * we don't want to use that as a starting point for each probe.
     */
    if (verbose_level > 2)
      SERIAL_PROTOCOLLNPGM("Positioning the probe...");

    // Disable bed level correction in M48 because we want the raw data when we probe
    #if HAS_ABL
      reset_bed_level();
    #endif

    setup_for_endstop_or_probe_move();

    // Move to the first point, deploy, and probe
    probe_pt(X_probe_location, Y_probe_location, stow_probe_after_each, verbose_level);

    randomSeed(millis());

    double mean = 0.0, sigma = 0.0, min = 99999.9, max = -99999.9, sample_set[n_samples];

    for (uint8_t n = 0; n < n_samples; n++) {
      if (n_legs) {
        int dir = (random(0, 10) > 5.0) ? -1 : 1;  // clockwise or counter clockwise
        float angle = random(0.0, 360.0),
              radius = random(
                #if ENABLED(DELTA)
                  DELTA_PROBEABLE_RADIUS / 8, DELTA_PROBEABLE_RADIUS / 3
                #else
                  5, X_MAX_LENGTH / 8
                #endif
              );

        if (verbose_level > 3) {
          SERIAL_ECHOPAIR("Starting radius: ", radius);
          SERIAL_ECHOPAIR("   angle: ", angle);
          SERIAL_ECHOPGM(" Direction: ");
          if (dir > 0) SERIAL_ECHOPGM("Counter-");
          SERIAL_ECHOLNPGM("Clockwise");
        }

        for (uint8_t l = 0; l < n_legs - 1; l++) {
          double delta_angle;

          if (schizoid_flag)
            // The points of a 5 point star are 72 degrees apart.  We need to
            // skip a point and go to the next one on the star.
            delta_angle = dir * 2.0 * 72.0;

          else
            // If we do this line, we are just trying to move further
            // around the circle.
            delta_angle = dir * (float) random(25, 45);

          angle += delta_angle;

          while (angle > 360.0)   // We probably do not need to keep the angle between 0 and 2*PI, but the
            angle -= 360.0;       // Arduino documentation says the trig functions should not be given values
          while (angle < 0.0)     // outside of this range.   It looks like they behave correctly with
            angle += 360.0;       // numbers outside of the range, but just to be safe we clamp them.

          X_current = X_probe_location - (X_PROBE_OFFSET_FROM_EXTRUDER) + cos(RADIANS(angle)) * radius;
          Y_current = Y_probe_location - (Y_PROBE_OFFSET_FROM_EXTRUDER) + sin(RADIANS(angle)) * radius;

          #if DISABLED(DELTA)
            X_current = constrain(X_current, X_MIN_POS, X_MAX_POS);
            Y_current = constrain(Y_current, Y_MIN_POS, Y_MAX_POS);
          #else
            // If we have gone out too far, we can do a simple fix and scale the numbers
            // back in closer to the origin.
            while (HYPOT(X_current, Y_current) > DELTA_PROBEABLE_RADIUS) {
              X_current /= 1.25;
              Y_current /= 1.25;
              if (verbose_level > 3) {
                SERIAL_ECHOPAIR("Pulling point towards center:", X_current);
                SERIAL_ECHOLNPAIR(", ", Y_current);
              }
            }
          #endif
          if (verbose_level > 3) {
            SERIAL_PROTOCOLPGM("Going to:");
            SERIAL_ECHOPAIR(" X", X_current);
            SERIAL_ECHOPAIR(" Y", Y_current);
            SERIAL_ECHOLNPAIR(" Z", current_position[Z_AXIS]);
          }
          do_blocking_move_to_xy(X_current, Y_current);
        } // n_legs loop
      } // n_legs

      // Probe a single point
      sample_set[n] = probe_pt(X_probe_location, Y_probe_location, stow_probe_after_each, 0);

      /**
       * Get the current mean for the data points we have so far
       */
      double sum = 0.0;
      for (uint8_t j = 0; j <= n; j++) sum += sample_set[j];
      mean = sum / (n + 1);

      NOMORE(min, sample_set[n]);
      NOLESS(max, sample_set[n]);

      /**
       * Now, use that mean to calculate the standard deviation for the
       * data points we have so far
       */
      sum = 0.0;
      for (uint8_t j = 0; j <= n; j++)
        sum += sq(sample_set[j] - mean);

      sigma = sqrt(sum / (n + 1));
      if (verbose_level > 0) {
        if (verbose_level > 1) {
          SERIAL_PROTOCOL(n + 1);
          SERIAL_PROTOCOLPGM(" of ");
          SERIAL_PROTOCOL((int)n_samples);
          SERIAL_PROTOCOLPGM(": z: ");
          SERIAL_PROTOCOL_F(sample_set[n], 3);
          if (verbose_level > 2) {
            SERIAL_PROTOCOLPGM(" mean: ");
            SERIAL_PROTOCOL_F(mean, 4);
            SERIAL_PROTOCOLPGM(" sigma: ");
            SERIAL_PROTOCOL_F(sigma, 6);
            SERIAL_PROTOCOLPGM(" min: ");
            SERIAL_PROTOCOL_F(min, 3);
            SERIAL_PROTOCOLPGM(" max: ");
            SERIAL_PROTOCOL_F(max, 3);
            SERIAL_PROTOCOLPGM(" range: ");
            SERIAL_PROTOCOL_F(max-min, 3);
          }
        }
        SERIAL_EOL;
      }

    } // End of probe loop

    if (STOW_PROBE()) return;

    SERIAL_PROTOCOLPGM("Finished!");
    SERIAL_EOL;

    if (verbose_level > 0) {
      SERIAL_PROTOCOLPGM("Mean: ");
      SERIAL_PROTOCOL_F(mean, 6);
      SERIAL_PROTOCOLPGM(" Min: ");
      SERIAL_PROTOCOL_F(min, 3);
      SERIAL_PROTOCOLPGM(" Max: ");
      SERIAL_PROTOCOL_F(max, 3);
      SERIAL_PROTOCOLPGM(" Range: ");
      SERIAL_PROTOCOL_F(max-min, 3);
      SERIAL_EOL;
    }

    SERIAL_PROTOCOLPGM("Standard Deviation: ");
    SERIAL_PROTOCOL_F(sigma, 6);
    SERIAL_EOL;
    SERIAL_EOL;

    clean_up_after_endstop_or_probe_move();

    report_current_position();
  }

#endif // Z_MIN_PROBE_REPEATABILITY_TEST

/**
 * M75: Start print timer
 */
inline void gcode_M75() { print_job_timer.start(); }

/**
 * M76: Pause print timer
 */
inline void gcode_M76() { print_job_timer.pause(); }

/**
 * M77: Stop print timer
 */
inline void gcode_M77() { print_job_timer.stop(); }

#if ENABLED(PRINTCOUNTER)
  /**
   * M78: Show print statistics
   */
  inline void gcode_M78() {
    // "M78 S78" will reset the statistics
    if (code_seen('S') && code_value_int() == 78)
      print_job_timer.initStats();
    else
      print_job_timer.showStats();
  }
#endif

//&begin[Hotend]
/**
 * M104: Set hot end temperature
 */
inline void gcode_M104() {
  if (get_target_extruder_from_command(104)) return;
  if (DEBUGGING(DRYRUN)) return;

  #if ENABLED(SINGLENOZZLE)
    if (target_extruder != active_extruder) return;
  #endif

  if (code_seen('S')) {
    thermalManager.setTargetHotend(code_value_temp_abs(), target_extruder);
    #if ENABLED(DUAL_X_CARRIAGE)
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        thermalManager.setTargetHotend(code_value_temp_abs() == 0.0 ? 0.0 : code_value_temp_abs() + duplicate_extruder_temp_offset, 1);
    #endif

    #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
      /**
       * Stop the timer at the end of print, starting is managed by
       * 'heat and wait' M109.
       * We use half EXTRUDE_MINTEMP here to allow nozzles to be put into hot
       * stand by mode, for instance in a dual extruder setup, without affecting
       * the running print timer.
       */
      if (code_value_temp_abs() <= (EXTRUDE_MINTEMP)/2) {
        print_job_timer.stop();
        LCD_MESSAGEPGM(WELCOME_MSG);
      }
    #endif

    if (code_value_temp_abs() > thermalManager.degHotend(target_extruder)) LCD_MESSAGEPGM(MSG_HEATING);
  }
  
  #if ENABLED(AUTOTEMP)
    planner.autotemp_M104_M109();
  #endif
}

//&begin[Bed_Heated]
#if HAS_TEMP_HOTEND || HAS_TEMP_BED

  void print_heaterstates() {
    #if HAS_TEMP_HOTEND
      SERIAL_PROTOCOLPGM(" T:");
      SERIAL_PROTOCOL_F(thermalManager.degHotend(target_extruder), 1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(thermalManager.degTargetHotend(target_extruder), 1);
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        SERIAL_PROTOCOLPAIR(" (", thermalManager.current_temperature_raw[target_extruder] / OVERSAMPLENR);
        SERIAL_CHAR(')');
      #endif
    #endif
    #if HAS_TEMP_BED
      SERIAL_PROTOCOLPGM(" B:");
      SERIAL_PROTOCOL_F(thermalManager.degBed(), 1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(thermalManager.degTargetBed(), 1);
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        SERIAL_PROTOCOLPAIR(" (", thermalManager.current_temperature_bed_raw / OVERSAMPLENR);
        SERIAL_CHAR(')');
      #endif
    #endif
    #if HOTENDS > 1
      HOTEND_LOOP() {
        SERIAL_PROTOCOLPAIR(" T", e);
        SERIAL_PROTOCOLCHAR(':');
        SERIAL_PROTOCOL_F(thermalManager.degHotend(e), 1);
        SERIAL_PROTOCOLPGM(" /");
        SERIAL_PROTOCOL_F(thermalManager.degTargetHotend(e), 1);
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          SERIAL_PROTOCOLPAIR(" (", thermalManager.current_temperature_raw[e] / OVERSAMPLENR);
          SERIAL_CHAR(')');
        #endif
      }
    #endif
    SERIAL_PROTOCOLPGM(" @:");
    SERIAL_PROTOCOL(thermalManager.getHeaterPower(target_extruder));
    #if HAS_TEMP_BED
      SERIAL_PROTOCOLPGM(" B@:");
      SERIAL_PROTOCOL(thermalManager.getHeaterPower(-1));
    #endif
    #if HOTENDS > 1
      HOTEND_LOOP() {
        SERIAL_PROTOCOLPAIR(" @", e);
        SERIAL_PROTOCOLCHAR(':');
        SERIAL_PROTOCOL(thermalManager.getHeaterPower(e));
      }
    #endif
  }
#endif

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105() {
  if (get_target_extruder_from_command(105)) return;

  #if HAS_TEMP_HOTEND || HAS_TEMP_BED
    SERIAL_PROTOCOLPGM(MSG_OK);
    print_heaterstates();
  #else // !HAS_TEMP_HOTEND && !HAS_TEMP_BED
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
  #endif

  SERIAL_EOL;
}
//&end[Hotend]
//&end[Bed_Heated]

#if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)

  static uint8_t auto_report_temp_interval;
  static millis_t next_temp_report_ms;

  /**
   * M155: Set temperature auto-report interval. M155 S<seconds>
   */
  inline void gcode_M155() {
    if (code_seen('S')) {
      auto_report_temp_interval = code_value_byte();
      NOMORE(auto_report_temp_interval, 60);
      next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
    }
  }

  inline void auto_report_temperatures() {
    if (auto_report_temp_interval && ELAPSED(millis(), next_temp_report_ms)) {
      next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
      print_heaterstates();
      SERIAL_EOL;
    }
  }

#endif // AUTO_REPORT_TEMPERATURES

//&begin[Fan]
#if FAN_COUNT > 0

  /**
   * M106: Set Fan Speed
   *
   *  S<int>   Speed between 0-255
   *  P<index> Fan index, if more than one fan
   */
  inline void gcode_M106() {
    uint16_t s = code_seen('S') ? code_value_ushort() : 255,
             p = code_seen('P') ? code_value_ushort() : 0;
    NOMORE(s, 255);
    if (p < FAN_COUNT) fanSpeeds[p] = s;
  }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107() {
    uint16_t p = code_seen('P') ? code_value_ushort() : 0;
    if (p < FAN_COUNT) fanSpeeds[p] = 0;
  }

#endif // FAN_COUNT > 0
//&end[Fan]

#if DISABLED(EMERGENCY_PARSER)

  /**
   * M108: Stop the waiting for heaters in M109, M190, M303. Does not affect the target temperature.
   */
  inline void gcode_M108() { wait_for_heatup = false; }


  /**
   * M112: Emergency Stop
   */
  inline void gcode_M112() { kill(PSTR(MSG_KILLED)); }


  /**
   * M410: Quickstop - Abort all planned moves
   *
   * This will stop the carriages mid-move, so most likely they
   * will be out of sync with the stepper position after this.
   */
  inline void gcode_M410() { quickstop_stepper(); }

#endif

//&begin[Hotend]
  #ifndef MIN_COOLING_SLOPE_DEG
    #define MIN_COOLING_SLOPE_DEG 1.50
  #endif
  #ifndef MIN_COOLING_SLOPE_TIME
    #define MIN_COOLING_SLOPE_TIME 60
  #endif

/**
 * M109: Sxxx Wait for extruder(s) to reach temperature. Waits only when heating.
 *       Rxxx Wait for extruder(s) to reach temperature. Waits when heating and cooling.
 */
inline void gcode_M109() {

  if (get_target_extruder_from_command(109)) return;
  if (DEBUGGING(DRYRUN)) return;

  #if ENABLED(SINGLENOZZLE)
    if (target_extruder != active_extruder) return;
  #endif

  bool no_wait_for_cooling = code_seen('S');
  if (no_wait_for_cooling || code_seen('R')) {
    thermalManager.setTargetHotend(code_value_temp_abs(), target_extruder);
    #if ENABLED(DUAL_X_CARRIAGE)
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        thermalManager.setTargetHotend(code_value_temp_abs() == 0.0 ? 0.0 : code_value_temp_abs() + duplicate_extruder_temp_offset, 1);
    #endif

    #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
      /**
       * We use half EXTRUDE_MINTEMP here to allow nozzles to be put into hot
       * stand by mode, for instance in a dual extruder setup, without affecting
       * the running print timer.
       */
      if (code_value_temp_abs() <= (EXTRUDE_MINTEMP)/2) {
        print_job_timer.stop();
        LCD_MESSAGEPGM(WELCOME_MSG);
      }
      /**
       * We do not check if the timer is already running because this check will
       * be done for us inside the Stopwatch::start() method thus a running timer
       * will not restart.
       */
      else print_job_timer.start();
    #endif

    if (thermalManager.isHeatingHotend(target_extruder)) LCD_MESSAGEPGM(MSG_HEATING);
  }

  #if ENABLED(AUTOTEMP)
    planner.autotemp_M104_M109();
  #endif

  #if TEMP_RESIDENCY_TIME > 0
    millis_t residency_start_ms = 0;
    // Loop until the temperature has stabilized
    #define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_RESIDENCY_TIME) * 1000UL))
  #else
    // Loop until the temperature is very close target
    #define TEMP_CONDITIONS (wants_to_cool ? thermalManager.isCoolingHotend(target_extruder) : thermalManager.isHeatingHotend(target_extruder))
  #endif //TEMP_RESIDENCY_TIME > 0

  float theTarget = -1.0, old_temp = 9999.0;
  bool wants_to_cool = false;
  wait_for_heatup = true;
  millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

  KEEPALIVE_STATE(NOT_BUSY);

  do {
    // Target temperature might be changed during the loop
    if (theTarget != thermalManager.degTargetHotend(target_extruder)) {
      wants_to_cool = thermalManager.isCoolingHotend(target_extruder);
      theTarget = thermalManager.degTargetHotend(target_extruder);

      // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
      if (no_wait_for_cooling && wants_to_cool) break;
    }

    now = millis();
    if (ELAPSED(now, next_temp_ms)) { //Print temp & remaining time every 1s while waiting
      next_temp_ms = now + 1000UL;
      print_heaterstates();
      #if TEMP_RESIDENCY_TIME > 0
        SERIAL_PROTOCOLPGM(" W:");
        if (residency_start_ms) {
          long rem = (((TEMP_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
          SERIAL_PROTOCOLLN(rem);
        }
        else {
          SERIAL_PROTOCOLLNPGM("?");
        }
      #else
        SERIAL_EOL;
      #endif
    }

    idle();
    refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

    float temp = thermalManager.degHotend(target_extruder);

    #if TEMP_RESIDENCY_TIME > 0

      float temp_diff = fabs(theTarget - temp);

      if (!residency_start_ms) {
        // Start the TEMP_RESIDENCY_TIME timer when we reach target temp for the first time.
        if (temp_diff < TEMP_WINDOW) residency_start_ms = now;
      }
      else if (temp_diff > TEMP_HYSTERESIS) {
        // Restart the timer whenever the temperature falls outside the hysteresis.
        residency_start_ms = now;
      }

    #endif //TEMP_RESIDENCY_TIME > 0

    // Prevent a wait-forever situation if R is misused i.e. M109 R0
    if (wants_to_cool) {
      // break after MIN_COOLING_SLOPE_TIME seconds
      // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG
      if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
        if (old_temp - temp < MIN_COOLING_SLOPE_DEG) break;
        next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME;
        old_temp = temp;
      }
    }

  } while (wait_for_heatup && TEMP_CONDITIONS);

  if (wait_for_heatup) LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);

  KEEPALIVE_STATE(IN_HANDLER);
}
//&end[Hotend]

//&begin[Bed_Heated]
#if HAS_TEMP_BED

  #ifndef MIN_COOLING_SLOPE_DEG_BED
    #define MIN_COOLING_SLOPE_DEG_BED 1.50
  #endif
  #ifndef MIN_COOLING_SLOPE_TIME_BED
    #define MIN_COOLING_SLOPE_TIME_BED 60
  #endif

  /**
   * M190: Sxxx Wait for bed current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M190() {
    if (DEBUGGING(DRYRUN)) return;

    LCD_MESSAGEPGM(MSG_BED_HEATING);
    bool no_wait_for_cooling = code_seen('S');
    if (no_wait_for_cooling || code_seen('R')) {
      thermalManager.setTargetBed(code_value_temp_abs());
      #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
        if (code_value_temp_abs() > BED_MINTEMP) {
          /**
          * We start the timer when 'heating and waiting' command arrives, LCD
          * functions never wait. Cooling down managed by extruders.
          *
          * We do not check if the timer is already running because this check will
          * be done for us inside the Stopwatch::start() method thus a running timer
          * will not restart.
          */
          print_job_timer.start();
        }
      #endif
    }

    #if TEMP_BED_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      // Loop until the temperature has stabilized
      #define TEMP_BED_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_BED_RESIDENCY_TIME) * 1000UL))
    #else
      // Loop until the temperature is very close target
      #define TEMP_BED_CONDITIONS (wants_to_cool ? thermalManager.isCoolingBed() : thermalManager.isHeatingBed())
    #endif //TEMP_BED_RESIDENCY_TIME > 0

    float theTarget = -1.0, old_temp = 9999.0;
    bool wants_to_cool = false;
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

    KEEPALIVE_STATE(NOT_BUSY);

    target_extruder = active_extruder; // for print_heaterstates

    do {
      // Target temperature might be changed during the loop
      if (theTarget != thermalManager.degTargetBed()) {
        wants_to_cool = thermalManager.isCoolingBed();
        theTarget = thermalManager.degTargetBed();

        // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
        if (no_wait_for_cooling && wants_to_cool) break;
      }

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { //Print Temp Reading every 1 second while heating up.
        next_temp_ms = now + 1000UL;
        print_heaterstates();
        #if TEMP_BED_RESIDENCY_TIME > 0
          SERIAL_PROTOCOLPGM(" W:");
          if (residency_start_ms) {
            long rem = (((TEMP_BED_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
            SERIAL_PROTOCOLLN(rem);
          }
          else {
            SERIAL_PROTOCOLLNPGM("?");
          }
        #else
          SERIAL_EOL;
        #endif
      }

      idle();
      refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

      float temp = thermalManager.degBed();

      #if TEMP_BED_RESIDENCY_TIME > 0

        float temp_diff = fabs(theTarget - temp);

        if (!residency_start_ms) {
          // Start the TEMP_BED_RESIDENCY_TIME timer when we reach target temp for the first time.
          if (temp_diff < TEMP_BED_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_BED_HYSTERESIS) {
          // Restart the timer whenever the temperature falls outside the hysteresis.
          residency_start_ms = now;
        }

      #endif //TEMP_BED_RESIDENCY_TIME > 0

      // Prevent a wait-forever situation if R is misused i.e. M190 R0
      if (wants_to_cool) {
        // break after MIN_COOLING_SLOPE_TIME_BED seconds
        // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG_BED
        if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
          if (old_temp - temp < MIN_COOLING_SLOPE_DEG_BED) break;
          next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME_BED;
          old_temp = temp;
        }
      }

    } while (wait_for_heatup && TEMP_BED_CONDITIONS);

    if (wait_for_heatup) LCD_MESSAGEPGM(MSG_BED_DONE);
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif // HAS_TEMP_BED
//&end[Bed_Heated]

//&begin[Command_Handling]
/**
 * M110: Set Current Line Number
 */
inline void gcode_M110() {
  if (code_seen('N')) gcode_N = code_value_long();
}
//&end[Command_Handling]

/**
 * M111: Set the debug level
 */
inline void gcode_M111() {
  marlin_debug_flags = code_seen('S') ? code_value_byte() : (uint8_t) DEBUG_NONE;

  const static char str_debug_1[] PROGMEM = MSG_DEBUG_ECHO;
  const static char str_debug_2[] PROGMEM = MSG_DEBUG_INFO;
  const static char str_debug_4[] PROGMEM = MSG_DEBUG_ERRORS;
  const static char str_debug_8[] PROGMEM = MSG_DEBUG_DRYRUN;
  const static char str_debug_16[] PROGMEM = MSG_DEBUG_COMMUNICATION;
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    const static char str_debug_32[] PROGMEM = MSG_DEBUG_LEVELING;
  #endif

  const static char* const debug_strings[] PROGMEM = {
    str_debug_1, str_debug_2, str_debug_4, str_debug_8, str_debug_16,
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      str_debug_32
    #endif
  };

  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_DEBUG_PREFIX);
  if (marlin_debug_flags) {
    uint8_t comma = 0;
    for (uint8_t i = 0; i < COUNT(debug_strings); i++) {
      if (TEST(marlin_debug_flags, i)) {
        if (comma++) SERIAL_CHAR(',');
        serialprintPGM((char*)pgm_read_word(&(debug_strings[i])));
      }
    }
  }
  else {
    SERIAL_ECHOPGM(MSG_DEBUG_OFF);
  }
  SERIAL_EOL;
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)

  /**
   * M113: Get or set Host Keepalive interval (0 to disable)
   *
   *   S<seconds> Optional. Set the keepalive interval.
   */
  inline void gcode_M113() {
    if (code_seen('S')) {
      host_keepalive_interval = code_value_byte();
      NOMORE(host_keepalive_interval, 60);
    }
    else {
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPAIR("M113 S", (unsigned long)host_keepalive_interval);
    }
  }

#endif

#if ENABLED(BARICUDA)

  #if HAS_HEATER_1
    /**
     * M126: Heater 1 valve open
     */
    inline void gcode_M126() { baricuda_valve_pressure = code_seen('S') ? code_value_byte() : 255; }
    /**
     * M127: Heater 1 valve close
     */
    inline void gcode_M127() { baricuda_valve_pressure = 0; }
  #endif

  #if HAS_HEATER_2
    /**
     * M128: Heater 2 valve open
     */
    inline void gcode_M128() { baricuda_e_to_p_pressure = code_seen('S') ? code_value_byte() : 255; }
    /**
     * M129: Heater 2 valve close
     */
    inline void gcode_M129() { baricuda_e_to_p_pressure = 0; }
  #endif

#endif //BARICUDA

//&begin[Bed_Heated]
/**
 * M140: Set bed temperature
 */
inline void gcode_M140() {
  if (DEBUGGING(DRYRUN)) return;
  if (code_seen('S')) thermalManager.setTargetBed(code_value_temp_abs());
}
//&end[Bed_Heated]

#if ENABLED(ULTIPANEL)

  /**
   * M145: Set the heatup state for a material in the LCD menu
   *   S<material> (0=PLA, 1=ABS)
   *   H<hotend temp>
   *   B<bed temp>
   *   F<fan speed>
   */
  inline void gcode_M145() {
    uint8_t material = code_seen('S') ? (uint8_t)code_value_int() : 0;
    if (material >= COUNT(lcd_preheat_hotend_temp)) {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM(MSG_ERR_MATERIAL_INDEX);
    }
    else {
      int v;
      if (code_seen('H')) {
        v = code_value_int();
        lcd_preheat_hotend_temp[material] = constrain(v, EXTRUDE_MINTEMP, HEATER_0_MAXTEMP - 15);
      }
      if (code_seen('F')) {
        v = code_value_int();
        lcd_preheat_fan_speed[material] = constrain(v, 0, 255);
      }
      #if TEMP_SENSOR_BED != 0
        if (code_seen('B')) {
          v = code_value_int();
          lcd_preheat_bed_temp[material] = constrain(v, BED_MINTEMP, BED_MAXTEMP - 15);
        }
      #endif
    }
  }

#endif // ULTIPANEL

//&begin[Temperature_Units_Support]
#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  /**
   * M149: Set temperature units
   */
  inline void gcode_M149() {
         if (code_seen('C')) set_input_temp_units(TEMPUNIT_C);
    else if (code_seen('K')) set_input_temp_units(TEMPUNIT_K);
    else if (code_seen('F')) set_input_temp_units(TEMPUNIT_F);
  }
#endif
//&end[Temperature_Units_Support]

//&begin[Power_Supply]
#if HAS_POWER_SWITCH

  /**
   * M80: Turn on Power Supply
   */
  inline void gcode_M80() {
    OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE); //GND

    /**
     * If you have a switch on suicide pin, this is useful
     * if you want to start another print with suicide feature after
     * a print without suicide...
     */
    #if HAS_SUICIDE
      OUT_WRITE(SUICIDE_PIN, HIGH);
    #endif

    #if ENABLED(ULTIPANEL)
      powersupply = true;
      LCD_MESSAGEPGM(WELCOME_MSG);
      lcd_update();
    #endif
  }

#endif // HAS_POWER_SWITCH

/**
 * M81: Turn off Power, including Power Supply, if there is one.
 *
 *      This code should ALWAYS be available for EMERGENCY SHUTDOWN!
 */
inline void gcode_M81() {
  thermalManager.disable_all_heaters();
  stepper.finish_and_disable();
  #if FAN_COUNT > 0
    #if FAN_COUNT > 1
      for (uint8_t i = 0; i < FAN_COUNT; i++) fanSpeeds[i] = 0;
    #else
      fanSpeeds[0] = 0;
    #endif
  #endif
  delay(1000); // Wait 1 second before switching off
  #if HAS_SUICIDE
    stepper.synchronize();
    suicide();
  #elif HAS_POWER_SWITCH
    OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
  #endif
  #if ENABLED(ULTIPANEL)
    #if HAS_POWER_SWITCH
      powersupply = false;
    #endif
    LCD_MESSAGEPGM(MACHINE_NAME " " MSG_OFF ".");
    lcd_update();
  #endif
}
//&end[Power_Supply]

//&begin[Extruder]
/**
 * M82: Set E codes absolute (default)
 */
inline void gcode_M82() { axis_relative_modes[E_AXIS] = false; }

/**
 * M83: Set E codes relative while in Absolute Coordinates (G90) mode
 */
inline void gcode_M83() { axis_relative_modes[E_AXIS] = true; }
//&end[Extruder]


//&begin[Moter_Type_Stepper]
/**
 * M18, M84: Disable all stepper motors
 */
inline void gcode_M18_M84() {
  if (code_seen('S')) {
    stepper_inactive_time = code_value_millis_from_seconds();
  }
  else {
    bool all_axis = !((code_seen('X')) || (code_seen('Y')) || (code_seen('Z')) || (code_seen('E')));
    if (all_axis) {
      stepper.finish_and_disable();
    }
    else {
      stepper.synchronize();
      if (code_seen('X')) disable_x();
      if (code_seen('Y')) disable_y();
      if (code_seen('Z')) disable_z();
      #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
        if (code_seen('E')) {
          disable_e0();
          disable_e1();
          disable_e2();
          disable_e3();
        }
      #endif
    }
  }
}
//&end[Moter_Type_Stepper]

/**
 * M85: Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 */
inline void gcode_M85() {
  if (code_seen('S')) max_inactive_time = code_value_millis_from_seconds();
}

//&begin[Moter_Type_Stepper]
/**
 * Multi-stepper support for M92, M201, M203
 */
#if ENABLED(DISTINCT_E_FACTORS)
  #define GET_TARGET_EXTRUDER(CMD) if (get_target_extruder_from_command(CMD)) return
  #define TARGET_EXTRUDER target_extruder
#else
  #define GET_TARGET_EXTRUDER(CMD) NOOP
  #define TARGET_EXTRUDER 0
#endif

/**
 * M92: Set axis steps-per-unit for one or more axes, X, Y, Z, and E.
 *      (Follows the same syntax as G92)
 *
 *      With multiple extruders use T to specify which one.
 */
inline void gcode_M92() {

  GET_TARGET_EXTRUDER(92);

  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i])) {
      if (i == E_AXIS) {
        float value = code_value_per_axis_unit(E_AXIS + TARGET_EXTRUDER);
        if (value < 20.0) {
          float factor = planner.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] / value; // increase e constants if M92 E14 is given for netfab.
          planner.max_jerk[E_AXIS] *= factor;
          planner.max_feedrate_mm_s[E_AXIS + TARGET_EXTRUDER] *= factor;
          planner.max_acceleration_steps_per_s2[E_AXIS + TARGET_EXTRUDER] *= factor;
        }
        planner.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] = value;
      }
      else {
        planner.axis_steps_per_mm[i] = code_value_per_axis_unit(i);
      }
    }
  }
  planner.refresh_positioning();
}

/**
 * Output the current position to serial
 */
static void report_current_position() {
  SERIAL_PROTOCOLPGM("X:");
  SERIAL_PROTOCOL(current_position[X_AXIS]);
  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(current_position[Y_AXIS]);
  SERIAL_PROTOCOLPGM(" Z:");
  SERIAL_PROTOCOL(current_position[Z_AXIS]);
  SERIAL_PROTOCOLPGM(" E:");
  SERIAL_PROTOCOL(current_position[E_AXIS]);

  stepper.report_positions();

  #if IS_SCARA
    SERIAL_PROTOCOLPAIR("SCARA Theta:", stepper.get_axis_position_degrees(A_AXIS));
    SERIAL_PROTOCOLLNPAIR("   Psi+Theta:", stepper.get_axis_position_degrees(B_AXIS));
    SERIAL_EOL;
  #endif
}

/**
 * M114: Output current position to serial port
 */
inline void gcode_M114() { report_current_position(); }
//&end[Moter_Type_Stepper]

/**
 * M115: Capabilities string
 */
inline void gcode_M115() {
  SERIAL_PROTOCOLLNPGM(MSG_M115_REPORT);

  #if ENABLED(EXTENDED_CAPABILITIES_REPORT)

    // EEPROM (M500, M501)
    #if ENABLED(EEPROM_SETTINGS)
      SERIAL_PROTOCOLLNPGM("Cap:EEPROM:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:EEPROM:0");
    #endif

    // AUTOREPORT_TEMP (M155)
    #if ENABLED(AUTO_REPORT_TEMPERATURES)
      SERIAL_PROTOCOLLNPGM("Cap:AUTOREPORT_TEMP:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:AUTOREPORT_TEMP:0");
    #endif

    // PROGRESS (M530 S L, M531 <file>, M532 X L)
    SERIAL_PROTOCOLLNPGM("Cap:PROGRESS:0");

    // AUTOLEVEL (G29)
    #if HAS_ABL
      SERIAL_PROTOCOLLNPGM("Cap:AUTOLEVEL:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:AUTOLEVEL:0");
    #endif

    // Z_PROBE (G30)
    #if HAS_BED_PROBE
      SERIAL_PROTOCOLLNPGM("Cap:Z_PROBE:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:Z_PROBE:0");
    #endif

    // SOFTWARE_POWER (G30)
    #if HAS_POWER_SWITCH
      SERIAL_PROTOCOLLNPGM("Cap:SOFTWARE_POWER:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:SOFTWARE_POWER:0");
    #endif

    // TOGGLE_LIGHTS (M355)
    #if HAS_CASE_LIGHT
      SERIAL_PROTOCOLLNPGM("Cap:TOGGLE_LIGHTS:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:TOGGLE_LIGHTS:0");
    #endif

    // EMERGENCY_PARSER (M108, M112, M410)
    #if ENABLED(EMERGENCY_PARSER)
      SERIAL_PROTOCOLLNPGM("Cap:EMERGENCY_PARSER:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:EMERGENCY_PARSER:0");
    #endif

  #endif // EXTENDED_CAPABILITIES_REPORT
}

/**
 * M117: Set LCD Status Message
 */
inline void gcode_M117() {
  lcd_setstatus(current_command_args);
}

/**
 * M119: Output endstop states to serial output
 */
//&begin[Endstop]
inline void gcode_M119() { endstops.M119(); }

/**
 * M120: Enable endstops and set non-homing endstop state to "enabled"
 */
inline void gcode_M120() { endstops.enable_globally(true); }

/**
 * M121: Disable endstops and set non-homing endstop state to "disabled"
 */
inline void gcode_M121() { endstops.enable_globally(false); }
//&end[Endstop]

#if ENABLED(HAVE_TMC2130DRIVER)

  /**
   * M122: Output Trinamic TMC2130 status to serial output. Very bad formatting.
   */

  static void tmc2130_report(Trinamic_TMC2130 &stepr, const char *name) {
    stepr.read_STAT();
    SERIAL_PROTOCOL(name);
    SERIAL_PROTOCOL(": ");
    stepr.isReset() ? SERIAL_PROTOCOLPGM("RESET ") : SERIAL_PROTOCOLPGM("----- ");
    stepr.isError() ? SERIAL_PROTOCOLPGM("ERROR ") : SERIAL_PROTOCOLPGM("----- ");
    stepr.isStallguard() ? SERIAL_PROTOCOLPGM("SLGRD ") : SERIAL_PROTOCOLPGM("----- ");
    stepr.isStandstill() ? SERIAL_PROTOCOLPGM("STILL ") : SERIAL_PROTOCOLPGM("----- ");
    SERIAL_PROTOCOLLN(stepr.debug());
  }

  inline void gcode_M122() {
    SERIAL_PROTOCOLLNPGM("Reporting TMC2130 status");
    #if ENABLED(X_IS_TMC2130)
      tmc2130_report(stepperX, "X");
    #endif
    #if ENABLED(X2_IS_TMC2130)
      tmc2130_report(stepperX2, "X2");
    #endif
    #if ENABLED(Y_IS_TMC2130)
      tmc2130_report(stepperY, "Y");
    #endif
    #if ENABLED(Y2_IS_TMC2130)
      tmc2130_report(stepperY2, "Y2");
    #endif
    #if ENABLED(Z_IS_TMC2130)
      tmc2130_report(stepperZ, "Z");
    #endif
    #if ENABLED(Z2_IS_TMC2130)
      tmc2130_report(stepperZ2, "Z2");
    #endif
    #if ENABLED(E0_IS_TMC2130)
      tmc2130_report(stepperE0, "E0");
    #endif
    #if ENABLED(E1_IS_TMC2130)
      tmc2130_report(stepperE1, "E1");
    #endif
    #if ENABLED(E2_IS_TMC2130)
      tmc2130_report(stepperE2, "E2");
    #endif
    #if ENABLED(E3_IS_TMC2130)
      tmc2130_report(stepperE3, "E3");
    #endif
  }
#endif // HAVE_TMC2130DRIVER

#if ENABLED(BLINKM) || ENABLED(RGB_LED)

  void set_led_color(const uint8_t r, const uint8_t g, const uint8_t b) {

    #if ENABLED(BLINKM)

      // This variant uses i2c to send the RGB components to the device.
      SendColors(r, g, b);

    #else

      // This variant uses 3 separate pins for the RGB components.
      // If the pins can do PWM then their intensity will be set.
      digitalWrite(RGB_LED_R_PIN, r ? HIGH : LOW);
      digitalWrite(RGB_LED_G_PIN, g ? HIGH : LOW);
      digitalWrite(RGB_LED_B_PIN, b ? HIGH : LOW);
      analogWrite(RGB_LED_R_PIN, r);
      analogWrite(RGB_LED_G_PIN, g);
      analogWrite(RGB_LED_B_PIN, b);

    #endif
  }

  /**
   * M150: Set Status LED Color - Use R-U-B for R-G-B
   *
   * Always sets all 3 components. If a component is left out, set to 0.
   *
   * Examples:
   *
   *   M150 R255       ; Turn LED red
   *   M150 R255 U127  ; Turn LED orange (PWM only)
   *   M150            ; Turn LED off
   *   M150 R U B      ; Turn LED white
   *
   */
  inline void gcode_M150() {
    set_led_color(
      code_seen('R') ? (code_has_value() ? code_value_byte() : 255) : 0,
      code_seen('U') ? (code_has_value() ? code_value_byte() : 255) : 0,
      code_seen('B') ? (code_has_value() ? code_value_byte() : 255) : 0
    );
  }

#endif // BLINKM || RGB_LED

//&begin[Extruder]
/**
 * M200: Set filament diameter and set E axis units to cubic units
 *
 *    T<extruder> - Optional extruder number. Current extruder if omitted.
 *    D<linear> - Diameter of the filament. Use "D0" to switch back to linear units on the E axis.
 */
inline void gcode_M200() {

  if (get_target_extruder_from_command(200)) return;

  if (code_seen('D')) {
    // setting any extruder filament size disables volumetric on the assumption that
    // slicers either generate in extruder values as cubic mm or as as filament feeds
    // for all extruders
    volumetric_enabled = (code_value_linear_units() != 0.0);
    if (volumetric_enabled) {
      filament_size[target_extruder] = code_value_linear_units();
      // make sure all extruders have some sane value for the filament size
      for (uint8_t i = 0; i < COUNT(filament_size); i++)
        if (! filament_size[i]) filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
    }
  }
  else {
    //reserved for setting filament diameter via UFID or filament measuring device
    return;
  }
  calculate_volumetric_multipliers();
}
//&end[Extruder]

//&begin[Moter_Type_Stepper]
/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M201() {

  GET_TARGET_EXTRUDER(201);

  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      planner.max_acceleration_mm_per_s2[a] = code_value_axis_units(a);
    }
  }
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  planner.reset_acceleration_rates();
}

#if 0 // Not used for Sprinter/grbl gen6
  inline void gcode_M202() {
    LOOP_XYZE(i) {
      if (code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value_axis_units(i) * planner.axis_steps_per_mm[i];
    }
  }
#endif
//&end[Moter_Type_Stepper]

//begin[Extruder]
/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in units/sec
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M203() {

  GET_TARGET_EXTRUDER(203);

  LOOP_XYZE(i)
    if (code_seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      planner.max_feedrate_mm_s[a] = code_value_axis_units(a);
    }
}
//end[Extruder]

/**
 * M204: Set Accelerations in units/sec^2 (M204 P1200 R3000 T3000)
 *
 *    P = Printing moves
 *    R = Retract only (no X, Y, Z) moves
 *    T = Travel (non printing) moves
 *
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
 */
//&begin[Moter_Type_Stepper]
inline void gcode_M204() {
  if (code_seen('S')) {  // Kept for legacy compatibility. Should NOT BE USED for new developments.
    planner.travel_acceleration = planner.acceleration = code_value_linear_units();
    SERIAL_ECHOLNPAIR("Setting Print and Travel Acceleration: ", planner.acceleration);
  }
  if (code_seen('P')) {
    planner.acceleration = code_value_linear_units();
    SERIAL_ECHOLNPAIR("Setting Print Acceleration: ", planner.acceleration);
  }
  if (code_seen('R')) {
    planner.retract_acceleration = code_value_linear_units();
    SERIAL_ECHOLNPAIR("Setting Retract Acceleration: ", planner.retract_acceleration);
  }
  if (code_seen('T')) {
    planner.travel_acceleration = code_value_linear_units();
    SERIAL_ECHOLNPAIR("Setting Travel Acceleration: ", planner.travel_acceleration);
  }
}

/**
 * M205: Set Advanced Settings
 *
 *    S = Min Feed Rate (units/s)
 *    T = Min Travel Feed Rate (units/s)
 *    B = Min Segment Time (µs)
 *    X = Max X Jerk (units/sec^2)
 *    Y = Max Y Jerk (units/sec^2)
 *    Z = Max Z Jerk (units/sec^2)
 *    E = Max E Jerk (units/sec^2)
 */
inline void gcode_M205() {
  if (code_seen('S')) planner.min_feedrate_mm_s = code_value_linear_units();
  if (code_seen('T')) planner.min_travel_feedrate_mm_s = code_value_linear_units();
  if (code_seen('B')) planner.min_segment_time = code_value_millis();
  if (code_seen('X')) planner.max_jerk[X_AXIS] = code_value_axis_units(X_AXIS);
  if (code_seen('Y')) planner.max_jerk[Y_AXIS] = code_value_axis_units(Y_AXIS);
  if (code_seen('Z')) planner.max_jerk[Z_AXIS] = code_value_axis_units(Z_AXIS);
  if (code_seen('E')) planner.max_jerk[E_AXIS] = code_value_axis_units(E_AXIS);
}
//&end[Moter_Type_Stepper]

//&begin[Homing]
/**
 * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
 */
inline void gcode_M206() {
  LOOP_XYZ(i)
    if (code_seen(axis_codes[i]))
      set_home_offset((AxisEnum)i, code_value_axis_units(i));

  #if ENABLED(MORGAN_SCARA)
    if (code_seen('T')) set_home_offset(A_AXIS, code_value_axis_units(A_AXIS)); // Theta
    if (code_seen('P')) set_home_offset(B_AXIS, code_value_axis_units(B_AXIS)); // Psi
  #endif

  SYNC_PLAN_POSITION_KINEMATIC();
  report_current_position();
}
//&end[Homing]

#if ENABLED(DELTA)
  /**
   * M665: Set delta configurations
   *
   *    L = diagonal rod
   *    R = delta radius
   *    S = segments per second
   *    A = Alpha (Tower 1) diagonal rod trim
   *    B = Beta (Tower 2) diagonal rod trim
   *    C = Gamma (Tower 3) diagonal rod trim
   */
  inline void gcode_M665() {
    if (code_seen('L')) delta_diagonal_rod = code_value_linear_units();
    if (code_seen('R')) delta_radius = code_value_linear_units();
    if (code_seen('S')) delta_segments_per_second = code_value_float();
    if (code_seen('A')) delta_diagonal_rod_trim_tower_1 = code_value_linear_units();
    if (code_seen('B')) delta_diagonal_rod_trim_tower_2 = code_value_linear_units();
    if (code_seen('C')) delta_diagonal_rod_trim_tower_3 = code_value_linear_units();
    recalc_delta_settings(delta_radius, delta_diagonal_rod);
  }
  /**
   * M666: Set delta endstop adjustment
   */
  inline void gcode_M666() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOLNPGM(">>> gcode_M666");
      }
    #endif
    LOOP_XYZ(i) {
      if (code_seen(axis_codes[i])) {
        endstop_adj[i] = code_value_axis_units(i);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_ECHOPAIR("endstop_adj[", axis_codes[i]);
            SERIAL_ECHOLNPAIR("] = ", endstop_adj[i]);
          }
        #endif
      }
    }
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOLNPGM("<<< gcode_M666");
      }
    #endif
  }

#elif ENABLED(Z_DUAL_ENDSTOPS) // !DELTA && ENABLED(Z_DUAL_ENDSTOPS)

  /**
   * M666: For Z Dual Endstop setup, set z axis offset to the z2 axis.
   */
  inline void gcode_M666() {
    if (code_seen('Z')) z_endstop_adj = code_value_axis_units(Z_AXIS);
    SERIAL_ECHOLNPAIR("Z Endstop Adjustment set to (mm):", z_endstop_adj);
  }

#endif // !DELTA && Z_DUAL_ENDSTOPS

#if ENABLED(FWRETRACT)

  /**
   * M207: Set firmware retraction values
   *
   *   S[+units]    retract_length
   *   W[+units]    retract_length_swap (multi-extruder)
   *   F[units/min] retract_feedrate_mm_s
   *   Z[units]     retract_zlift
   */
  inline void gcode_M207() {
    if (code_seen('S')) retract_length = code_value_axis_units(E_AXIS);
    if (code_seen('F')) retract_feedrate_mm_s = MMM_TO_MMS(code_value_axis_units(E_AXIS));
    if (code_seen('Z')) retract_zlift = code_value_axis_units(Z_AXIS);
    #if EXTRUDERS > 1
      if (code_seen('W')) retract_length_swap = code_value_axis_units(E_AXIS);
    #endif
  }

  /**
   * M208: Set firmware un-retraction values
   *
   *   S[+units]    retract_recover_length (in addition to M207 S*)
   *   W[+units]    retract_recover_length_swap (multi-extruder)
   *   F[units/min] retract_recover_feedrate_mm_s
   */
  inline void gcode_M208() {
    if (code_seen('S')) retract_recover_length = code_value_axis_units(E_AXIS);
    if (code_seen('F')) retract_recover_feedrate_mm_s = MMM_TO_MMS(code_value_axis_units(E_AXIS));
    #if EXTRUDERS > 1
      if (code_seen('W')) retract_recover_length_swap = code_value_axis_units(E_AXIS);
    #endif
  }

  /**
   * M209: Enable automatic retract (M209 S1)
   *   For slicers that don't support G10/11, reversed extrude-only
   *   moves will be classified as retraction.
   */
  inline void gcode_M209() {
    if (code_seen('S')) {
      autoretract_enabled = code_value_bool();
      for (int i = 0; i < EXTRUDERS; i++) retracted[i] = false;
    }
  }

#endif // FWRETRACT

//&begin[Endstop]
/**
 * M211: Enable, Disable, and/or Report software endstops
 *
 * Usage: M211 S1 to enable, M211 S0 to disable, M211 alone for report
 */
inline void gcode_M211() {
  SERIAL_ECHO_START;
  #if ENABLED(min_software_endstops) || ENABLED(max_software_endstops)
    if (code_seen('S')) soft_endstops_enabled = code_value_bool();
  #endif
  #if ENABLED(min_software_endstops) || ENABLED(max_software_endstops)
    SERIAL_ECHOPGM(MSG_SOFT_ENDSTOPS);
    serialprintPGM(soft_endstops_enabled ? PSTR(MSG_ON) : PSTR(MSG_OFF));
  #else
    SERIAL_ECHOPGM(MSG_SOFT_ENDSTOPS);
    SERIAL_ECHOPGM(MSG_OFF);
  #endif
  SERIAL_ECHOPGM(MSG_SOFT_MIN);
  SERIAL_ECHOPAIR(    MSG_X, soft_endstop_min[X_AXIS]);
  SERIAL_ECHOPAIR(" " MSG_Y, soft_endstop_min[Y_AXIS]);
  SERIAL_ECHOPAIR(" " MSG_Z, soft_endstop_min[Z_AXIS]);
  SERIAL_ECHOPGM(MSG_SOFT_MAX);
  SERIAL_ECHOPAIR(    MSG_X, soft_endstop_max[X_AXIS]);
  SERIAL_ECHOPAIR(" " MSG_Y, soft_endstop_max[Y_AXIS]);
  SERIAL_ECHOLNPAIR(" " MSG_Z, soft_endstop_max[Z_AXIS]);
}
//&end[Endstop]

//&begin[Hotend]
#if HOTENDS > 1

  /**
   * M218 - set hotend offset (in linear units)
   *
   *   T<tool>
   *   X<xoffset>
   *   Y<yoffset>
   *   Z<zoffset> - Available with DUAL_X_CARRIAGE and SWITCHING_EXTRUDER
   */
  inline void gcode_M218() {
    if (get_target_extruder_from_command(218) || target_extruder == 0) return;

    if (code_seen('X')) hotend_offset[X_AXIS][target_extruder] = code_value_axis_units(X_AXIS);
    if (code_seen('Y')) hotend_offset[Y_AXIS][target_extruder] = code_value_axis_units(Y_AXIS);

    #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(SWITCHING_EXTRUDER)
      if (code_seen('Z')) hotend_offset[Z_AXIS][target_extruder] = code_value_axis_units(Z_AXIS);
    #endif

    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
    HOTEND_LOOP() {
      SERIAL_CHAR(' ');
      SERIAL_ECHO(hotend_offset[X_AXIS][e]);
      SERIAL_CHAR(',');
      SERIAL_ECHO(hotend_offset[Y_AXIS][e]);
      #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(SWITCHING_EXTRUDER)
        SERIAL_CHAR(',');
        SERIAL_ECHO(hotend_offset[Z_AXIS][e]);
      #endif
    }
    SERIAL_EOL;
  }

#endif // HOTENDS > 1
//&end[Hotend]

//&begin[Extruder]
/**
 * M220: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 */
inline void gcode_M220() {
  if (code_seen('S')) feedrate_percentage = code_value_int();
}

/**
 * M221: Set extrusion percentage (M221 T0 S95)
 */
inline void gcode_M221() {
  if (get_target_extruder_from_command(221)) return;
  if (code_seen('S'))
    flow_percentage[target_extruder] = code_value_int();
}
//&end[Extruder]

//&begin[Board]
/**
 * M226: Wait until the specified pin reaches the state required (M226 P<pin> S<state>)
 */
inline void gcode_M226() {
  if (code_seen('P')) {
    int pin_number = code_value_int(),
        pin_state = code_seen('S') ? code_value_int() : -1; // required pin state - default is inverted

    if (pin_state >= -1 && pin_state <= 1 && pin_number > -1 && !pin_is_protected(pin_number)) {

      int target = LOW;

      stepper.synchronize();

      pinMode(pin_number, INPUT);
      switch (pin_state) {
        case 1:
          target = HIGH;
          break;
        case 0:
          target = LOW;
          break;
        case -1:
          target = !digitalRead(pin_number);
          break;
      }

      while (digitalRead(pin_number) != target) idle();

    } // pin_state -1 0 1 && pin_number > -1
  } // code_seen('P')
}
//&end[Board]

#if ENABLED(EXPERIMENTAL_I2CBUS)

  /**
   * M260: Send data to a I2C slave device
   *
   * This is a PoC, the formating and arguments for the GCODE will
   * change to be more compatible, the current proposal is:
   *
   *  M260 A<slave device address base 10> ; Sets the I2C slave address the data will be sent to
   *
   *  M260 B<byte-1 value in base 10>
   *  M260 B<byte-2 value in base 10>
   *  M260 B<byte-3 value in base 10>
   *
   *  M260 S1 ; Send the buffered data and reset the buffer
   *  M260 R1 ; Reset the buffer without sending data
   *
   */
  inline void gcode_M260() {
    // Set the target address
    if (code_seen('A')) i2c.address(code_value_byte());

    // Add a new byte to the buffer
    if (code_seen('B')) i2c.addbyte(code_value_byte());

    // Flush the buffer to the bus
    if (code_seen('S')) i2c.send();

    // Reset and rewind the buffer
    else if (code_seen('R')) i2c.reset();
  }

  /**
   * M261: Request X bytes from I2C slave device
   *
   * Usage: M261 A<slave device address base 10> B<number of bytes>
   */
  inline void gcode_M261() {
    if (code_seen('A')) i2c.address(code_value_byte());

    uint8_t bytes = code_seen('B') ? code_value_byte() : 1;

    if (i2c.addr && bytes && bytes <= TWIBUS_BUFFER_SIZE) {
      i2c.relay(bytes);
    }
    else {
      SERIAL_ERROR_START;
      SERIAL_ERRORLN("Bad i2c request");
    }
  }

#endif // EXPERIMENTAL_I2CBUS

//&begin[Moter_Type_Servo]
#if HAS_SERVOS

  /**
   * M280: Get or set servo position. P<index> [S<angle>]
   */
  inline void gcode_M280() {
    if (!code_seen('P')) return;
    int servo_index = code_value_int();
    if (servo_index >= 0 && servo_index < NUM_SERVOS) {
      if (code_seen('S'))
        MOVE_SERVO(servo_index, code_value_int());
      else {
        SERIAL_ECHO_START;
        SERIAL_ECHOPAIR(" Servo ", servo_index);
        SERIAL_ECHOLNPAIR(": ", servo[servo_index].read());
      }
    }
    else {
      SERIAL_ERROR_START;
      SERIAL_ECHOPAIR("Servo ", servo_index);
      SERIAL_ECHOLNPGM(" out of range");
    }
  }

#endif // HAS_SERVOS
//&end[Moter_Type_Servo]

//&begin[Buzzer]
#if HAS_BUZZER

  /**
   * M300: Play beep sound S<frequency Hz> P<duration ms>
   */
  inline void gcode_M300() {
    uint16_t const frequency = code_seen('S') ? code_value_ushort() : 260;
    uint16_t duration = code_seen('P') ? code_value_ushort() : 1000;

    // Limits the tone duration to 0-5 seconds.
    NOMORE(duration, 5000);

    BUZZ(duration, frequency);
  }

#endif // HAS_BUZZER
//&end[Buzzer]

//&begin[Hotend]
#if ENABLED(PIDTEMP)

  /**
   * M301: Set PID parameters P I D (and optionally C, L)
   *
   *   P[float] Kp term
   *   I[float] Ki term (unscaled)
   *   D[float] Kd term (unscaled)
   *
   * With PID_EXTRUSION_SCALING:
   *
   *   C[float] Kc term
   *   L[float] LPQ length
   */
  inline void gcode_M301() {

    // multi-extruder PID patch: M301 updates or prints a single extruder's PID values
    // default behaviour (omitting E parameter) is to update for extruder 0 only
    int e = code_seen('E') ? code_value_int() : 0; // extruder being updated

    if (e < HOTENDS) { // catch bad input value
      if (code_seen('P')) PID_PARAM(Kp, e) = code_value_float();
      if (code_seen('I')) PID_PARAM(Ki, e) = scalePID_i(code_value_float());
      if (code_seen('D')) PID_PARAM(Kd, e) = scalePID_d(code_value_float());
      #if ENABLED(PID_EXTRUSION_SCALING)
        if (code_seen('C')) PID_PARAM(Kc, e) = code_value_float();
        if (code_seen('L')) lpq_len = code_value_float();
        NOMORE(lpq_len, LPQ_MAX_LEN);
      #endif

      thermalManager.updatePID();
      SERIAL_ECHO_START;
      #if ENABLED(PID_PARAMS_PER_HOTEND)
        SERIAL_ECHOPAIR(" e:", e); // specify extruder in serial output
      #endif // PID_PARAMS_PER_HOTEND
      SERIAL_ECHOPAIR(" p:", PID_PARAM(Kp, e));
      SERIAL_ECHOPAIR(" i:", unscalePID_i(PID_PARAM(Ki, e)));
      SERIAL_ECHOPAIR(" d:", unscalePID_d(PID_PARAM(Kd, e)));
      #if ENABLED(PID_EXTRUSION_SCALING)
        //Kc does not have scaling applied above, or in resetting defaults
        SERIAL_ECHOPAIR(" c:", PID_PARAM(Kc, e));
      #endif
      SERIAL_EOL;
    }
    else {
      SERIAL_ERROR_START;
      SERIAL_ERRORLN(MSG_INVALID_EXTRUDER);
    }
  }

#endif // PIDTEMP
//&end[Hotend]

//&begin[Bed_Heated]
#if ENABLED(PIDTEMPBED)

  inline void gcode_M304() {
    if (code_seen('P')) thermalManager.bedKp = code_value_float();
    if (code_seen('I')) thermalManager.bedKi = scalePID_i(code_value_float());
    if (code_seen('D')) thermalManager.bedKd = scalePID_d(code_value_float());

    thermalManager.updatePID();

    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR(" p:", thermalManager.bedKp);
    SERIAL_ECHOPAIR(" i:", unscalePID_i(thermalManager.bedKi));
    SERIAL_ECHOLNPAIR(" d:", unscalePID_d(thermalManager.bedKd));
  }

#endif // PIDTEMPBED
//&end[Bed_Heated]

#if defined(CHDK) || HAS_PHOTOGRAPH

  /**
   * M240: Trigger a camera by emulating a Canon RC-1
   *       See http://www.doc-diy.net/photo/rc-1_hacked/
   */
  inline void gcode_M240() {
    #ifdef CHDK

      OUT_WRITE(CHDK, HIGH);
      chdkHigh = millis();
      chdkActive = true;

    #elif HAS_PHOTOGRAPH

      const uint8_t NUM_PULSES = 16;
      const float PULSE_LENGTH = 0.01524;
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }
      delay(7.33);
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }

    #endif // !CHDK && HAS_PHOTOGRAPH
  }

#endif // CHDK || PHOTOGRAPH_PIN

#if HAS_LCD_CONTRAST

  /**
   * M250: Read and optionally set the LCD contrast
   */
  inline void gcode_M250() {
    if (code_seen('C')) set_lcd_contrast(code_value_int());
    SERIAL_PROTOCOLPGM("lcd contrast value: ");
    SERIAL_PROTOCOL(lcd_contrast);
    SERIAL_EOL;
  }

#endif // HAS_LCD_CONTRAST

#if ENABLED(PREVENT_COLD_EXTRUSION)

  /**
   * M302: Allow cold extrudes, or set the minimum extrude temperature
   *
   *       S<temperature> sets the minimum extrude temperature
   *       P<bool> enables (1) or disables (0) cold extrusion
   *
   *  Examples:
   *
   *       M302         ; report current cold extrusion state
   *       M302 P0      ; enable cold extrusion checking
   *       M302 P1      ; disables cold extrusion checking
   *       M302 S0      ; always allow extrusion (disables checking)
   *       M302 S170    ; only allow extrusion above 170
   *       M302 S170 P1 ; set min extrude temp to 170 but leave disabled
   */
  inline void gcode_M302() {
    bool seen_S = code_seen('S');
    if (seen_S) {
      thermalManager.extrude_min_temp = code_value_temp_abs();
      thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0);
    }

    if (code_seen('P'))
      thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0) || code_value_bool();
    else if (!seen_S) {
      // Report current state
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("Cold extrudes are ", (thermalManager.allow_cold_extrude ? "en" : "dis"));
      SERIAL_ECHOPAIR("abled (min temp ", int(thermalManager.extrude_min_temp + 0.5));
      SERIAL_ECHOLNPGM("C)");
    }
  }

#endif // PREVENT_COLD_EXTRUSION

/**
 * M303: PID relay autotune
 *
 *       S<temperature> sets the target temperature. (default 150C)
 *       E<extruder> (-1 for the bed) (default 0)
 *       C<cycles>
 *       U<bool> with a non-zero value will apply the result to current settings
 */
inline void gcode_M303() {
  #if HAS_PID_HEATING
    int e = code_seen('E') ? code_value_int() : 0;
    int c = code_seen('C') ? code_value_int() : 5;
    bool u = code_seen('U') && code_value_bool();

    float temp = code_seen('S') ? code_value_temp_abs() : (e < 0 ? 70.0 : 150.0);

    if (e >= 0 && e < HOTENDS)
      target_extruder = e;

    KEEPALIVE_STATE(NOT_BUSY); // don't send "busy: processing" messages during autotune output

    thermalManager.PID_autotune(temp, e, c, u);

    KEEPALIVE_STATE(IN_HANDLER);
  #else
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_M303_DISABLED);
  #endif
}

#if ENABLED(MORGAN_SCARA)

  bool SCARA_move_to_cal(uint8_t delta_a, uint8_t delta_b) {
    if (IsRunning()) {
      forward_kinematics_SCARA(delta_a, delta_b);
      destination[X_AXIS] = LOGICAL_X_POSITION(cartes[X_AXIS]);
      destination[Y_AXIS] = LOGICAL_Y_POSITION(cartes[Y_AXIS]);
      destination[Z_AXIS] = current_position[Z_AXIS];
      prepare_move_to_destination();
      return true;
    }
    return false;
  }

  /**
   * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
   */
  inline bool gcode_M360() {
    SERIAL_ECHOLNPGM(" Cal: Theta 0");
    return SCARA_move_to_cal(0, 120);
  }

  /**
   * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M361() {
    SERIAL_ECHOLNPGM(" Cal: Theta 90");
    return SCARA_move_to_cal(90, 130);
  }

  /**
   * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
   */
  inline bool gcode_M362() {
    SERIAL_ECHOLNPGM(" Cal: Psi 0");
    return SCARA_move_to_cal(60, 180);
  }

  /**
   * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M363() {
    SERIAL_ECHOLNPGM(" Cal: Psi 90");
    return SCARA_move_to_cal(50, 90);
  }

  /**
   * M364: SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
   */
  inline bool gcode_M364() {
    SERIAL_ECHOLNPGM(" Cal: Theta-Psi 90");
    return SCARA_move_to_cal(45, 135);
  }

#endif // SCARA

#if ENABLED(EXT_SOLENOID)

  void enable_solenoid(uint8_t num) {
    switch (num) {
      case 0:
        OUT_WRITE(SOL0_PIN, HIGH);
        break;
        #if HAS_SOLENOID_1
          case 1:
            OUT_WRITE(SOL1_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_2
          case 2:
            OUT_WRITE(SOL2_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_3
          case 3:
            OUT_WRITE(SOL3_PIN, HIGH);
            break;
        #endif
      default:
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_INVALID_SOLENOID);
        break;
    }
  }

  void enable_solenoid_on_active_extruder() { enable_solenoid(active_extruder); }

  void disable_all_solenoids() {
    OUT_WRITE(SOL0_PIN, LOW);
    OUT_WRITE(SOL1_PIN, LOW);
    OUT_WRITE(SOL2_PIN, LOW);
    OUT_WRITE(SOL3_PIN, LOW);
  }

  /**
   * M380: Enable solenoid on the active extruder
   */
  inline void gcode_M380() { enable_solenoid_on_active_extruder(); }

  /**
   * M381: Disable all solenoids
   */
  inline void gcode_M381() { disable_all_solenoids(); }

#endif // EXT_SOLENOID

//&begin[Move_To_Destination]
/**
 * M400: Finish all moves
 */
inline void gcode_M400() { stepper.synchronize(); }
//&end[Move_To_Destination]

#if HAS_BED_PROBE

  /**
   * M401: Engage Z Servo endstop if available
   */
  inline void gcode_M401() { DEPLOY_PROBE(); }

  /**
   * M402: Retract Z Servo endstop if enabled
   */
  inline void gcode_M402() { STOW_PROBE(); }

#endif // HAS_BED_PROBE

#if ENABLED(FILAMENT_WIDTH_SENSOR)

  /**
   * M404: Display or set (in current units) the nominal filament width (3mm, 1.75mm ) W<3.0>
   */
  inline void gcode_M404() {
    if (code_seen('W')) {
      filament_width_nominal = code_value_linear_units();
    }
    else {
      SERIAL_PROTOCOLPGM("Filament dia (nominal mm):");
      SERIAL_PROTOCOLLN(filament_width_nominal);
    }
  }

  /**
   * M405: Turn on filament sensor for control
   */
  inline void gcode_M405() {
    // This is technically a linear measurement, but since it's quantized to centimeters and is a different unit than
    // everything else, it uses code_value_int() instead of code_value_linear_units().
    if (code_seen('D')) meas_delay_cm = code_value_int();
    NOMORE(meas_delay_cm, MAX_MEASUREMENT_DELAY);

    if (filwidth_delay_index[1] == -1) { // Initialize the ring buffer if not done since startup
      int temp_ratio = thermalManager.widthFil_to_size_ratio();

      for (uint8_t i = 0; i < COUNT(measurement_delay); ++i)
        measurement_delay[i] = temp_ratio - 100;  // Subtract 100 to scale within a signed byte

      filwidth_delay_index[0] = filwidth_delay_index[1] = 0;
    }

    filament_sensor = true;

    //SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
    //SERIAL_PROTOCOL(filament_width_meas);
    //SERIAL_PROTOCOLPGM("Extrusion ratio(%):");
    //SERIAL_PROTOCOL(flow_percentage[active_extruder]);
  }

  /**
   * M406: Turn off filament sensor for control
   */
  inline void gcode_M406() { filament_sensor = false; }

  /**
   * M407: Get measured filament diameter on serial output
   */
  inline void gcode_M407() {
    SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
    SERIAL_PROTOCOLLN(filament_width_meas);
  }

#endif // FILAMENT_WIDTH_SENSOR

//&begin[Moter_Type_Stepper]
void quickstop_stepper() {
  stepper.quick_stop();
  stepper.synchronize();
  set_current_from_steppers_for_axis(ALL_AXES);
  SYNC_PLAN_POSITION_KINEMATIC();
}
//&end[Moter_Type_Stepper]

#if PLANNER_LEVELING
  /**
   * M420: Enable/Disable Bed Leveling and/or set the Z fade height.
   *
   *       S[bool]   Turns leveling on or off
   *       Z[height] Sets the Z fade height (0 or none to disable)
   */
  inline void gcode_M420() {
    if (code_seen('S')) set_bed_leveling_enabled(code_value_bool());
    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      if (code_seen('Z')) set_z_fade_height(code_value_linear_units());
    #endif
  }
#endif

#if ENABLED(MESH_BED_LEVELING)

  /**
   * M421: Set a single Mesh Bed Leveling Z coordinate
   * Use either 'M421 X<linear> Y<linear> Z<linear>' or 'M421 I<xindex> J<yindex> Z<linear>'
   */
  inline void gcode_M421() {
    int8_t px = 0, py = 0;
    float z = 0;
    bool hasX, hasY, hasZ, hasI, hasJ;
    if ((hasX = code_seen('X'))) px = mbl.probe_index_x(code_value_axis_units(X_AXIS));
    if ((hasY = code_seen('Y'))) py = mbl.probe_index_y(code_value_axis_units(Y_AXIS));
    if ((hasI = code_seen('I'))) px = code_value_axis_units(X_AXIS);
    if ((hasJ = code_seen('J'))) py = code_value_axis_units(Y_AXIS);
    if ((hasZ = code_seen('Z'))) z = code_value_axis_units(Z_AXIS);

    if (hasX && hasY && hasZ) {

      if (px >= 0 && py >= 0)
        mbl.set_z(px, py, z);
      else {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_ERR_MESH_XY);
      }
    }
    else if (hasI && hasJ && hasZ) {
      if (px >= 0 && px < MESH_NUM_X_POINTS && py >= 0 && py < MESH_NUM_Y_POINTS)
        mbl.set_z(px, py, z);
      else {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_ERR_MESH_XY);
      }
    }
    else {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM(MSG_ERR_M421_PARAMETERS);
    }
  }

#endif

//&begin[Homing]
/**
 * M428: Set home_offset based on the distance between the
 *       current_position and the nearest "reference point."
 *       If an axis is past center its endstop position
 *       is the reference-point. Otherwise it uses 0. This allows
 *       the Z offset to be set near the bed when using a max endstop.
 *
 *       M428 can't be used more than 2cm away from 0 or an endstop.
 *
 *       Use M206 to set these values directly.
 */
inline void gcode_M428() {
  bool err = false;
  LOOP_XYZ(i) {
    if (axis_homed[i]) {
      float base = (current_position[i] > (soft_endstop_min[i] + soft_endstop_max[i]) * 0.5) ? base_home_pos((AxisEnum)i) : 0,
            diff = current_position[i] - LOGICAL_POSITION(base, i);
      if (diff > -20 && diff < 20) {
        set_home_offset((AxisEnum)i, home_offset[i] - diff);
      }
      else {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_ERR_M428_TOO_FAR);
        LCD_ALERTMESSAGEPGM("Err: Too far!");
        BUZZ(200, 40);
        err = true;
        break;
      }
    }
  }

  if (!err) {
    SYNC_PLAN_POSITION_KINEMATIC();
    report_current_position();
    LCD_MESSAGEPGM(MSG_HOME_OFFSETS_APPLIED);
    BUZZ(200, 659);
    BUZZ(200, 698);
  }
}
//&end[Homing]