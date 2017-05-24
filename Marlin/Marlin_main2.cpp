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
//&begin[Clean_Nozzle]
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
//&end[Clean_Nozzle]
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

//&begin[Park_Nozzle]
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
//&end[Park_Nozzle]

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
//&begin[Emergency_Command_Parser]
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
//&end[Emergency_Command_Parser]
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