/**
 * M500: Store settings in EEPROM
 */
inline void gcode_M500() {
  Config_StoreSettings();
}

/**
 * M501: Read settings from EEPROM
 */
inline void gcode_M501() {
  Config_RetrieveSettings();
}

/**
 * M502: Revert to default settings
 */
inline void gcode_M502() {
  Config_ResetDefault();
}

/**
 * M503: print settings currently in memory
 */
inline void gcode_M503() {
  Config_PrintSettings(code_seen('S') && !code_value_bool());
}

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)

  /**
   * M540: Set whether SD card print should abort on endstop hit (M540 S<0|1>)
   */
  inline void gcode_M540() {
    if (code_seen('S')) stepper.abort_on_endstop_hit = code_value_bool();
  }

#endif // ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

#if HAS_BED_PROBE

  inline void gcode_M851() {

    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_ZPROBE_ZOFFSET);
    SERIAL_CHAR(' ');

    if (code_seen('Z')) {
      float value = code_value_axis_units(Z_AXIS);
      if (Z_PROBE_OFFSET_RANGE_MIN <= value && value <= Z_PROBE_OFFSET_RANGE_MAX) {
        zprobe_zoffset = value;
        SERIAL_ECHO(zprobe_zoffset);
      }
      else {
        SERIAL_ECHOPAIR(MSG_Z_MIN, Z_PROBE_OFFSET_RANGE_MIN);
        SERIAL_CHAR(' ');
        SERIAL_ECHOPAIR(MSG_Z_MAX, Z_PROBE_OFFSET_RANGE_MAX);
      }
    }
    else {
      SERIAL_ECHOPAIR(": ", zprobe_zoffset);
    }

    SERIAL_EOL;
  }

#endif // HAS_BED_PROBE
//&begin[FILAMENT_CHANGE_FEATURE]
#if ENABLED(FILAMENT_CHANGE_FEATURE)

  /**
   * M600: Pause for filament change
   *
   *  E[distance] - Retract the filament this far (negative value)
   *  Z[distance] - Move the Z axis by this distance
   *  X[position] - Move to this X position, with Y
   *  Y[position] - Move to this Y position, with X
   *  L[distance] - Retract distance for removal (manual reload)
   *
   *  Default values are used for omitted arguments.
   *
   */
  inline void gcode_M600() {

    if (thermalManager.tooColdToExtrude(active_extruder)) {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM(MSG_TOO_COLD_FOR_M600);
      return;
    }

    // Show initial message and wait for synchronize steppers
    lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_INIT);
    stepper.synchronize();

    float lastpos[NUM_AXIS];

    // Save current position of all axes
    LOOP_XYZE(i)
      lastpos[i] = destination[i] = current_position[i];

    // Define runplan for move axes
    #if IS_KINEMATIC
      #define RUNPLAN(RATE_MM_S) planner.buffer_line_kinematic(destination, RATE_MM_S, active_extruder);
    #else
      #define RUNPLAN(RATE_MM_S) line_to_destination(RATE_MM_S);
    #endif

    KEEPALIVE_STATE(IN_HANDLER);

    // Initial retract before move to filament change position
    if (code_seen('E')) destination[E_AXIS] += code_value_axis_units(E_AXIS);
    #if defined(FILAMENT_CHANGE_RETRACT_LENGTH) && FILAMENT_CHANGE_RETRACT_LENGTH > 0
      else destination[E_AXIS] -= FILAMENT_CHANGE_RETRACT_LENGTH;
    #endif

    RUNPLAN(FILAMENT_CHANGE_RETRACT_FEEDRATE);

    // Lift Z axis
    float z_lift = code_seen('Z') ? code_value_axis_units(Z_AXIS) :
      #if defined(FILAMENT_CHANGE_Z_ADD) && FILAMENT_CHANGE_Z_ADD > 0
        FILAMENT_CHANGE_Z_ADD
      #else
        0
      #endif
    ;

    if (z_lift > 0) {
      destination[Z_AXIS] += z_lift;
      NOMORE(destination[Z_AXIS], Z_MAX_POS);
      RUNPLAN(FILAMENT_CHANGE_Z_FEEDRATE);
    }

    // Move XY axes to filament exchange position
    if (code_seen('X')) destination[X_AXIS] = code_value_axis_units(X_AXIS);
    #ifdef FILAMENT_CHANGE_X_POS
      else destination[X_AXIS] = FILAMENT_CHANGE_X_POS;
    #endif

    if (code_seen('Y')) destination[Y_AXIS] = code_value_axis_units(Y_AXIS);
    #ifdef FILAMENT_CHANGE_Y_POS
      else destination[Y_AXIS] = FILAMENT_CHANGE_Y_POS;
    #endif

    RUNPLAN(FILAMENT_CHANGE_XY_FEEDRATE);

    stepper.synchronize();
    lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_UNLOAD);

    // Unload filament
    if (code_seen('L')) destination[E_AXIS] += code_value_axis_units(E_AXIS);
    #if defined(FILAMENT_CHANGE_UNLOAD_LENGTH) && FILAMENT_CHANGE_UNLOAD_LENGTH > 0
      else destination[E_AXIS] -= FILAMENT_CHANGE_UNLOAD_LENGTH;
    #endif

    RUNPLAN(FILAMENT_CHANGE_UNLOAD_FEEDRATE);

    // Synchronize steppers and then disable extruders steppers for manual filament changing
    stepper.synchronize();
    disable_e0();
    disable_e1();
    disable_e2();
    disable_e3();
    delay(100);

    #if HAS_BUZZER
      millis_t next_buzz = 0;
    #endif

    // Wait for filament insert by user and press button
    lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_INSERT);

    // LCD click or M108 will clear this
    wait_for_user = true;

    while (wait_for_user) {
      #if HAS_BUZZER
        millis_t ms = millis();
        if (ms >= next_buzz) {
          BUZZ(300, 2000);
          next_buzz = ms + 2500; // Beep every 2.5s while waiting
        }
      #endif
      idle(true);
    }

    // Show load message
    lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_LOAD);

    // Load filament
    if (code_seen('L')) destination[E_AXIS] -= code_value_axis_units(E_AXIS);
    #if defined(FILAMENT_CHANGE_LOAD_LENGTH) && FILAMENT_CHANGE_LOAD_LENGTH > 0
      else destination[E_AXIS] += FILAMENT_CHANGE_LOAD_LENGTH;
    #endif

    RUNPLAN(FILAMENT_CHANGE_LOAD_FEEDRATE);
    stepper.synchronize();

    #if defined(FILAMENT_CHANGE_EXTRUDE_LENGTH) && FILAMENT_CHANGE_EXTRUDE_LENGTH > 0
      do {
        // Extrude filament to get into hotend
        lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_EXTRUDE);
        destination[E_AXIS] += FILAMENT_CHANGE_EXTRUDE_LENGTH;
        RUNPLAN(FILAMENT_CHANGE_EXTRUDE_FEEDRATE);
        stepper.synchronize();
        // Ask user if more filament should be extruded
        KEEPALIVE_STATE(PAUSED_FOR_USER);
        lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_OPTION);
        while (filament_change_menu_response == FILAMENT_CHANGE_RESPONSE_WAIT_FOR) idle(true);
        KEEPALIVE_STATE(IN_HANDLER);
      } while (filament_change_menu_response != FILAMENT_CHANGE_RESPONSE_RESUME_PRINT);
    #endif

    lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_RESUME);

    KEEPALIVE_STATE(IN_HANDLER);

    // Set extruder to saved position
    destination[E_AXIS] = current_position[E_AXIS] = lastpos[E_AXIS];
    planner.set_e_position_mm(current_position[E_AXIS]);

    #if IS_KINEMATIC
      // Move XYZ to starting position
      planner.buffer_line_kinematic(lastpos, FILAMENT_CHANGE_XY_FEEDRATE, active_extruder);
    #else
      // Move XY to starting position, then Z
      destination[X_AXIS] = lastpos[X_AXIS];
      destination[Y_AXIS] = lastpos[Y_AXIS];
      RUNPLAN(FILAMENT_CHANGE_XY_FEEDRATE);
      destination[Z_AXIS] = lastpos[Z_AXIS];
      RUNPLAN(FILAMENT_CHANGE_Z_FEEDRATE);
    #endif
    stepper.synchronize();

    #if ENABLED(FILAMENT_RUNOUT_SENSOR)
      filament_ran_out = false;
    #endif

    // Show status screen
    lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_STATUS);
  }

#endif // FILAMENT_CHANGE_FEATURE
//&end[FILAMENT_CHANGE_FEATURE]
#if ENABLED(DUAL_X_CARRIAGE)

  /**
   * M605: Set dual x-carriage movement mode
   *
   *    M605 S0: Full control mode. The slicer has full control over x-carriage movement
   *    M605 S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
   *    M605 S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
   *                         units x-offset and an optional differential hotend temperature of
   *                         mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
   *                         the first with a spacing of 100mm in the x direction and 2 degrees hotter.
   *
   *    Note: the X axis should be homed after changing dual x-carriage mode.
   */
  inline void gcode_M605() {
    stepper.synchronize();
    if (code_seen('S')) dual_x_carriage_mode = (DualXMode)code_value_byte();
    switch (dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE:
      case DXC_AUTO_PARK_MODE:
        break;
      case DXC_DUPLICATION_MODE:
        if (code_seen('X')) duplicate_extruder_x_offset = max(code_value_axis_units(X_AXIS), X2_MIN_POS - x_home_pos(0));
        if (code_seen('R')) duplicate_extruder_temp_offset = code_value_temp_diff();
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
        SERIAL_CHAR(' ');
        SERIAL_ECHO(hotend_offset[X_AXIS][0]);
        SERIAL_CHAR(',');
        SERIAL_ECHO(hotend_offset[Y_AXIS][0]);
        SERIAL_CHAR(' ');
        SERIAL_ECHO(duplicate_extruder_x_offset);
        SERIAL_CHAR(',');
        SERIAL_ECHOLN(hotend_offset[Y_AXIS][1]);
        break;
      default:
        dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        break;
    }
    active_extruder_parked = false;
    extruder_duplication_enabled = false;
    delayed_move_time = 0;
  }

#elif ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)

  inline void gcode_M605() {
    stepper.synchronize();
    extruder_duplication_enabled = code_seen('S') && code_value_int() == 2;
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPAIR(MSG_DUPLICATION_MODE, extruder_duplication_enabled ? MSG_ON : MSG_OFF);
  }

#endif // M605
//&begin[LIN_ADVANCE]
#if ENABLED(LIN_ADVANCE)
  /**
   * M905: Set advance factor
   */
  inline void gcode_M905() {
    stepper.synchronize();
    planner.advance_M905(code_seen('K') ? code_value_float() : -1.0);
  }
#endif
//&end[LIN_ADVANCE]
/**
 * M907: Set digital trimpot motor current using axis codes X, Y, Z, E, B, S
 */
inline void gcode_M907() {
  #if HAS_DIGIPOTSS
    LOOP_XYZE(i)
    if (code_seen(axis_codes[i])) stepper.digipot_current(i, code_value_int());
    if (code_seen('B')) stepper.digipot_current(4, code_value_int());
    if (code_seen('S')) for (int i = 0; i <= 4; i++) stepper.digipot_current(i, code_value_int());
  #elif HAS_MOTOR_CURRENT_PWM
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
      if (code_seen('X')) stepper.digipot_current(0, code_value_int());
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
      if (code_seen('Z')) stepper.digipot_current(1, code_value_int());
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
      if (code_seen('E')) stepper.digipot_current(2, code_value_int());
    #endif
  #endif
  #if ENABLED(DIGIPOT_I2C)
    // this one uses actual amps in floating point
    LOOP_XYZE(i) if (code_seen(axis_codes[i])) digipot_i2c_set_current(i, code_value_float());
    // for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
    for (int i = NUM_AXIS; i < DIGIPOT_I2C_NUM_CHANNELS; i++) if (code_seen('B' + i - (NUM_AXIS))) digipot_i2c_set_current(i, code_value_float());
  #endif
  #if ENABLED(DAC_STEPPER_CURRENT)
    if (code_seen('S')) {
      float dac_percent = code_value_float();
      for (uint8_t i = 0; i <= 4; i++) dac_current_percent(i, dac_percent);
    }
    LOOP_XYZE(i) if (code_seen(axis_codes[i])) dac_current_percent(i, code_value_float());
  #endif
}

#if HAS_DIGIPOTSS || ENABLED(DAC_STEPPER_CURRENT)

  /**
   * M908: Control digital trimpot directly (M908 P<pin> S<current>)
   */
  inline void gcode_M908() {
    #if HAS_DIGIPOTSS
      stepper.digitalPotWrite(
        code_seen('P') ? code_value_int() : 0,
        code_seen('S') ? code_value_int() : 0
      );
    #endif
    #ifdef DAC_STEPPER_CURRENT
      dac_current_raw(
        code_seen('P') ? code_value_byte() : -1,
        code_seen('S') ? code_value_ushort() : 0
      );
    #endif
  }

  #if ENABLED(DAC_STEPPER_CURRENT) // As with Printrbot RevF

    inline void gcode_M909() { dac_print_values(); }

    inline void gcode_M910() { dac_commit_eeprom(); }

  #endif

#endif // HAS_DIGIPOTSS || DAC_STEPPER_CURRENT

#if HAS_MICROSTEPS

  // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
  inline void gcode_M350() {
    if (code_seen('S')) for (int i = 0; i <= 4; i++) stepper.microstep_mode(i, code_value_byte());
    LOOP_XYZE(i) if (code_seen(axis_codes[i])) stepper.microstep_mode(i, code_value_byte());
    if (code_seen('B')) stepper.microstep_mode(4, code_value_byte());
    stepper.microstep_readings();
  }

  /**
   * M351: Toggle MS1 MS2 pins directly with axis codes X Y Z E B
   *       S# determines MS1 or MS2, X# sets the pin high/low.
   */
  inline void gcode_M351() {
    if (code_seen('S')) switch (code_value_byte()) {
      case 1:
        LOOP_XYZE(i) if (code_seen(axis_codes[i])) stepper.microstep_ms(i, code_value_byte(), -1);
        if (code_seen('B')) stepper.microstep_ms(4, code_value_byte(), -1);
        break;
      case 2:
        LOOP_XYZE(i) if (code_seen(axis_codes[i])) stepper.microstep_ms(i, -1, code_value_byte());
        if (code_seen('B')) stepper.microstep_ms(4, -1, code_value_byte());
        break;
    }
    stepper.microstep_readings();
  }

#endif // HAS_MICROSTEPS

//&begin[CASE_LIGHT]
#if HAS_CASE_LIGHT

  uint8_t case_light_brightness = 255;

  void update_case_light() {
    digitalWrite(CASE_LIGHT_PIN, case_light_on != INVERT_CASE_LIGHT ? HIGH : LOW);
    analogWrite(CASE_LIGHT_PIN, case_light_on != INVERT_CASE_LIGHT ? case_light_brightness : 0);
  }

  /**
   * M355: Turn case lights on/off and set brightness
   *
   *   S<bool>  Turn case light on or off
   *   P<byte>  Set case light brightness (PWM pin required)
   */
  inline void gcode_M355() {
    if (code_seen('P')) case_light_brightness = code_value_byte();
    if (code_seen('S')) case_light_on = code_value_bool();
    update_case_light();
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Case lights ");
    case_light_on ? SERIAL_ECHOLNPGM("on") : SERIAL_ECHOLNPGM("off");
  }

#endif // HAS_CASE_LIGHT
//&end[CASE_LIGHT]

//&begin[Extruder_Mixing]
#if ENABLED(MIXING_EXTRUDER)

  /**
   * M163: Set a single mix factor for a mixing extruder
   *       This is called "weight" by some systems.
   *
   *   S[index]   The channel index to set
   *   P[float]   The mix value
   *
   */
  inline void gcode_M163() {
    int mix_index = code_seen('S') ? code_value_int() : 0;
    if (mix_index < MIXING_STEPPERS) {
      float mix_value = code_seen('P') ? code_value_float() : 0.0;
      NOLESS(mix_value, 0.0);
      mixing_factor[mix_index] = RECIPROCAL(mix_value);
    }
  }

  #if MIXING_VIRTUAL_TOOLS > 1

    /**
     * M164: Store the current mix factors as a virtual tool.
     *
     *   S[index]   The virtual tool to store
     *
     */
    inline void gcode_M164() {
      int tool_index = code_seen('S') ? code_value_int() : 0;
      if (tool_index < MIXING_VIRTUAL_TOOLS) {
        normalize_mix();
        for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
          mixing_virtual_tool_mix[tool_index][i] = mixing_factor[i];
      }
    }

  #endif

  #if ENABLED(DIRECT_MIXING_IN_G1)
    /**
     * M165: Set multiple mix factors for a mixing extruder.
     *       Factors that are left out will be set to 0.
     *       All factors together must add up to 1.0.
     *
     *   A[factor] Mix factor for extruder stepper 1
     *   B[factor] Mix factor for extruder stepper 2
     *   C[factor] Mix factor for extruder stepper 3
     *   D[factor] Mix factor for extruder stepper 4
     *   H[factor] Mix factor for extruder stepper 5
     *   I[factor] Mix factor for extruder stepper 6
     *
     */
    inline void gcode_M165() { gcode_get_mix(); }
  #endif

#endif // MIXING_EXTRUDER
//&end[Extruder_Mixing]

/**
 * M999: Restart after being stopped
 *
 * Default behaviour is to flush the serial buffer and request
 * a resend to the host starting on the last N line received.
 *
 * Sending "M999 S1" will resume printing without flushing the
 * existing command buffer.
 *
 */
inline void gcode_M999() {
  Running = true;
  lcd_reset_alert_level();

  if (code_seen('S') && code_value_bool()) return;

  // gcode_LastN = Stopped_gcode_LastN;
  FlushSerialRequestResend();
}

//&begin[Extruder_Switching]
#if ENABLED(SWITCHING_EXTRUDER)
  inline void move_extruder_servo(uint8_t e) {
    const int angles[2] = SWITCHING_EXTRUDER_SERVO_ANGLES;
    MOVE_SERVO(SWITCHING_EXTRUDER_SERVO_NR, angles[e]);
  }
#endif
//&end[Extruder_Switching]

//&begin[Extruder]
inline void invalid_extruder_error(const uint8_t &e) {
  SERIAL_ECHO_START;
  SERIAL_CHAR('T');
  SERIAL_PROTOCOL_F(e, DEC);
  SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
}
//&end[Extruder]

//&begin[Extruder_Mixing]
//&begin[Extruder_Switching]
/**
 * Perform a tool-change, which may result in moving the
 * previous tool out of the way and the new tool into place.
 */
void tool_change(const uint8_t tmp_extruder, const float fr_mm_s/*=0.0*/, bool no_move/*=false*/) {
  #if ENABLED(MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1

    if (tmp_extruder >= MIXING_VIRTUAL_TOOLS)
      return invalid_extruder_error(tmp_extruder);

    // T0-Tnnn: Switch virtual tool by changing the mix
    for (uint8_t j = 0; j < MIXING_STEPPERS; j++)
      mixing_factor[j] = mixing_virtual_tool_mix[tmp_extruder][j];

  #else //!MIXING_EXTRUDER || MIXING_VIRTUAL_TOOLS <= 1

    #if HOTENDS > 1

      if (tmp_extruder >= EXTRUDERS)
        return invalid_extruder_error(tmp_extruder);

      float old_feedrate_mm_s = feedrate_mm_s;

      feedrate_mm_s = fr_mm_s > 0.0 ? (old_feedrate_mm_s = fr_mm_s) : XY_PROBE_FEEDRATE_MM_S;

      if (tmp_extruder != active_extruder) {
        if (!no_move && axis_unhomed_error(true, true, true)) {
          SERIAL_ECHOLNPGM("No move on toolchange");
          no_move = true;
        }

        // Save current position to destination, for use later
        set_destination_to_current();

        #if ENABLED(DUAL_X_CARRIAGE)

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOPGM("Dual X Carriage Mode ");
              switch (dual_x_carriage_mode) {
                case DXC_FULL_CONTROL_MODE: SERIAL_ECHOLNPGM("DXC_FULL_CONTROL_MODE"); break;
                case DXC_AUTO_PARK_MODE: SERIAL_ECHOLNPGM("DXC_AUTO_PARK_MODE"); break;
                case DXC_DUPLICATION_MODE: SERIAL_ECHOLNPGM("DXC_DUPLICATION_MODE"); break;
              }
            }
          #endif

          const float xhome = x_home_pos(active_extruder);
          if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE
              && IsRunning()
              && (delayed_move_time || current_position[X_AXIS] != xhome)
          ) {
            float raised_z = current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT;
            #if ENABLED(max_software_endstops)
              NOMORE(raised_z, soft_endstop_max[Z_AXIS]);
            #endif
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) {
                SERIAL_ECHOLNPAIR("Raise to ", raised_z);
                SERIAL_ECHOLNPAIR("MoveX to ", xhome);
                SERIAL_ECHOLNPAIR("Lower to ", current_position[Z_AXIS]);
              }
            #endif
            // Park old head: 1) raise 2) move to park position 3) lower
            for (uint8_t i = 0; i < 3; i++)
              planner.buffer_line(
                i == 0 ? current_position[X_AXIS] : xhome,
                current_position[Y_AXIS],
                i == 2 ? current_position[Z_AXIS] : raised_z,
                current_position[E_AXIS],
                planner.max_feedrate_mm_s[i == 1 ? X_AXIS : Z_AXIS],
                active_extruder
              );
            stepper.synchronize();
          }

          // Apply Y & Z extruder offset (X offset is used as home pos with Dual X)
          current_position[Y_AXIS] -= hotend_offset[Y_AXIS][active_extruder] - hotend_offset[Y_AXIS][tmp_extruder];
          current_position[Z_AXIS] -= hotend_offset[Z_AXIS][active_extruder] - hotend_offset[Z_AXIS][tmp_extruder];

          // Activate the new extruder
          active_extruder = tmp_extruder;

          // This function resets the max/min values - the current position may be overwritten below.
          set_axis_is_at_home(X_AXIS);

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("New Extruder", current_position);
          #endif

          // Only when auto-parking are carriages safe to move
          if (dual_x_carriage_mode != DXC_AUTO_PARK_MODE) no_move = true;

          switch (dual_x_carriage_mode) {
            case DXC_FULL_CONTROL_MODE:
              // New current position is the position of the activated extruder
              current_position[X_AXIS] = LOGICAL_X_POSITION(inactive_extruder_x_pos);
              // Save the inactive extruder's position (from the old current_position)
              inactive_extruder_x_pos = RAW_X_POSITION(destination[X_AXIS]);
              break;
            case DXC_AUTO_PARK_MODE:
              // record raised toolhead position for use by unpark
              memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
              raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
              #if ENABLED(max_software_endstops)
                NOMORE(raised_parked_position[Z_AXIS], soft_endstop_max[Z_AXIS]);
              #endif
              active_extruder_parked = true;
              delayed_move_time = 0;
              break;
            case DXC_DUPLICATION_MODE:
              // If the new extruder is the left one, set it "parked"
              // This triggers the second extruder to move into the duplication position
              active_extruder_parked = (active_extruder == 0);

              if (active_extruder_parked)
                current_position[X_AXIS] = LOGICAL_X_POSITION(inactive_extruder_x_pos);
              else
                current_position[X_AXIS] = destination[X_AXIS] + duplicate_extruder_x_offset;
              inactive_extruder_x_pos = RAW_X_POSITION(destination[X_AXIS]);
              extruder_duplication_enabled = false;
              break;
          }

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOLNPAIR("Active extruder parked: ", active_extruder_parked ? "yes" : "no");
              DEBUG_POS("New extruder (parked)", current_position);
            }
          #endif

          // No extra case for HAS_ABL in DUAL_X_CARRIAGE. Does that mean they don't work together?
        #else // !DUAL_X_CARRIAGE

          #if ENABLED(SWITCHING_EXTRUDER)
            // <0 if the new nozzle is higher, >0 if lower. A bigger raise when lower.
            float z_diff = hotend_offset[Z_AXIS][active_extruder] - hotend_offset[Z_AXIS][tmp_extruder],
                  z_raise = 0.3 + (z_diff > 0.0 ? z_diff : 0.0);

            // Always raise by some amount (destination copied from current_position earlier)
            destination[Z_AXIS] += z_raise;
            planner.buffer_line_kinematic(destination, planner.max_feedrate_mm_s[Z_AXIS], active_extruder);
            stepper.synchronize();

            move_extruder_servo(active_extruder);
            delay(500);

            // Move back down, if needed
            if (z_raise != z_diff) {
              destination[Z_AXIS] = current_position[Z_AXIS] + z_diff;
              planner.buffer_line_kinematic(destination, planner.max_feedrate_mm_s[Z_AXIS], active_extruder);
              stepper.synchronize();
            }
          #endif

          /**
           * Set current_position to the position of the new nozzle.
           * Offsets are based on linear distance, so we need to get
           * the resulting position in coordinate space.
           *
           * - With grid or 3-point leveling, offset XYZ by a tilted vector
           * - With mesh leveling, update Z for the new position
           * - Otherwise, just use the raw linear distance
           *
           * Software endstops are altered here too. Consider a case where:
           *   E0 at X=0 ... E1 at X=10
           * When we switch to E1 now X=10, but E1 can't move left.
           * To express this we apply the change in XY to the software endstops.
           * E1 can move farther right than E0, so the right limit is extended.
           *
           * Note that we don't adjust the Z software endstops. Why not?
           * Consider a case where Z=0 (here) and switching to E1 makes Z=1
           * because the bed is 1mm lower at the new position. As long as
           * the first nozzle is out of the way, the carriage should be
           * allowed to move 1mm lower. This technically "breaks" the
           * Z software endstop. But this is technically correct (and
           * there is no viable alternative).
           */
          #if ABL_PLANAR
            // Offset extruder, make sure to apply the bed level rotation matrix
            vector_3 tmp_offset_vec = vector_3(hotend_offset[X_AXIS][tmp_extruder],
                                               hotend_offset[Y_AXIS][tmp_extruder],
                                               0),
                     act_offset_vec = vector_3(hotend_offset[X_AXIS][active_extruder],
                                               hotend_offset[Y_AXIS][active_extruder],
                                               0),
                     offset_vec = tmp_offset_vec - act_offset_vec;

            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) {
                tmp_offset_vec.debug("tmp_offset_vec");
                act_offset_vec.debug("act_offset_vec");
                offset_vec.debug("offset_vec (BEFORE)");
              }
            #endif

            offset_vec.apply_rotation(planner.bed_level_matrix.transpose(planner.bed_level_matrix));

            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) offset_vec.debug("offset_vec (AFTER)");
            #endif

            // Adjustments to the current position
            float xydiff[2] = { offset_vec.x, offset_vec.y };
            current_position[Z_AXIS] += offset_vec.z;

          #else // !ABL_PLANAR

            float xydiff[2] = {
              hotend_offset[X_AXIS][tmp_extruder] - hotend_offset[X_AXIS][active_extruder],
              hotend_offset[Y_AXIS][tmp_extruder] - hotend_offset[Y_AXIS][active_extruder]
            };

            #if ENABLED(MESH_BED_LEVELING)

              if (mbl.active()) {
                #if ENABLED(DEBUG_LEVELING_FEATURE)
                  if (DEBUGGING(LEVELING)) SERIAL_ECHOPAIR("Z before MBL: ", current_position[Z_AXIS]);
                #endif
                float x2 = current_position[X_AXIS] + xydiff[X_AXIS],
                      y2 = current_position[Y_AXIS] + xydiff[Y_AXIS],
                      z1 = current_position[Z_AXIS], z2 = z1;
                planner.apply_leveling(current_position[X_AXIS], current_position[Y_AXIS], z1);
                planner.apply_leveling(x2, y2, z2);
                current_position[Z_AXIS] += z2 - z1;
                #if ENABLED(DEBUG_LEVELING_FEATURE)
                  if (DEBUGGING(LEVELING))
                    SERIAL_ECHOLNPAIR(" after: ", current_position[Z_AXIS]);
                #endif
              }

            #endif // MESH_BED_LEVELING

          #endif // !HAS_ABL

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOPAIR("Offset Tool XY by { ", xydiff[X_AXIS]);
              SERIAL_ECHOPAIR(", ", xydiff[Y_AXIS]);
              SERIAL_ECHOLNPGM(" }");
            }
          #endif

          // The newly-selected extruder XY is actually at...
          current_position[X_AXIS] += xydiff[X_AXIS];
          current_position[Y_AXIS] += xydiff[Y_AXIS];
          for (uint8_t i = X_AXIS; i <= Y_AXIS; i++) {
            position_shift[i] += xydiff[i];
            update_software_endstops((AxisEnum)i);
          }

          // Set the new active extruder
          active_extruder = tmp_extruder;

        #endif // !DUAL_X_CARRIAGE

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("Sync After Toolchange", current_position);
        #endif

        // Tell the planner the new "current position"
        SYNC_PLAN_POSITION_KINEMATIC();

        // Move to the "old position" (move the extruder into place)
        if (!no_move && IsRunning()) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("Move back", destination);
          #endif
          prepare_move_to_destination();
        }

      } // (tmp_extruder != active_extruder)

      stepper.synchronize();

      #if ENABLED(EXT_SOLENOID)
        disable_all_solenoids();
        enable_solenoid_on_active_extruder();
      #endif // EXT_SOLENOID

      feedrate_mm_s = old_feedrate_mm_s;

    #else // HOTENDS <= 1

      // Set the new active extruder
      active_extruder = tmp_extruder;

      UNUSED(fr_mm_s);
      UNUSED(no_move);

    #endif // HOTENDS <= 1

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPAIR(MSG_ACTIVE_EXTRUDER, (int)active_extruder);

  #endif //!MIXING_EXTRUDER || MIXING_VIRTUAL_TOOLS <= 1
}
//&end[Extruder_Mixing]
//&end[Extruder_Switching]

//&begin[Extruder_Switching]
/**
 * T0-T3: Switch tool, usually switching extruders
 *
 *   F[units/min] Set the movement feedrate
 *   S1           Don't move the tool in XY after change
 */
inline void gcode_T(uint8_t tmp_extruder) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR(">>> gcode_T(", tmp_extruder);
      SERIAL_CHAR(')');
      SERIAL_EOL;
      DEBUG_POS("BEFORE", current_position);
    }
  #endif

  #if HOTENDS == 1 || (ENABLED(MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1)

    tool_change(tmp_extruder);

  #elif HOTENDS > 1

    tool_change(
      tmp_extruder,
      code_seen('F') ? MMM_TO_MMS(code_value_axis_units(X_AXIS)) : 0.0,
      (tmp_extruder == active_extruder) || (code_seen('S') && code_value_bool())
    );

  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      DEBUG_POS("AFTER", current_position);
      SERIAL_ECHOLNPGM("<<< gcode_T");
    }
  #endif
}
//&end[Extruder_Switching]

//&begin[Command_Input_Process]
/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void process_next_command() {
  current_command = command_queue[cmd_queue_index_r];

  if (DEBUGGING(ECHO)) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLN(current_command);
  }

  // Sanitize the current command:
  //  - Skip leading spaces
  //  - Bypass N[-0-9][0-9]*[ ]*
  //  - Overwrite * with nul to mark the end
  while (*current_command == ' ') ++current_command;
  if (*current_command == 'N' && NUMERIC_SIGNED(current_command[1])) {
    current_command += 2; // skip N[-0-9]
    while (NUMERIC(*current_command)) ++current_command; // skip [0-9]*
    while (*current_command == ' ') ++current_command; // skip [ ]*
  }
  char* starpos = strchr(current_command, '*');  // * should always be the last parameter
  if (starpos) while (*starpos == ' ' || *starpos == '*') *starpos-- = '\0'; // nullify '*' and ' '

  char *cmd_ptr = current_command;

  // Get the command code, which must be G, M, or T
  char command_code = *cmd_ptr++;

  // Skip spaces to get the numeric part
  while (*cmd_ptr == ' ') cmd_ptr++;

  // Allow for decimal point in command
  #if ENABLED(G38_PROBE_TARGET)
    uint8_t subcode = 0;
  #endif

  uint16_t codenum = 0; // define ahead of goto

  // Bail early if there's no code
  bool code_is_good = NUMERIC(*cmd_ptr);
  if (!code_is_good) goto ExitUnknownCommand;

  // Get and skip the code number
  do {
    codenum = (codenum * 10) + (*cmd_ptr - '0');
    cmd_ptr++;
  } while (NUMERIC(*cmd_ptr));

  // Allow for decimal point in command
  #if ENABLED(G38_PROBE_TARGET)
    if (*cmd_ptr == '.') {
      cmd_ptr++;
      while (NUMERIC(*cmd_ptr))
        subcode = (subcode * 10) + (*cmd_ptr++ - '0');
    }
  #endif

  // Skip all spaces to get to the first argument, or nul
  while (*cmd_ptr == ' ') cmd_ptr++;

  // The command's arguments (if any) start here, for sure!
  current_command_args = cmd_ptr;

  KEEPALIVE_STATE(IN_HANDLER);

  // Handle a known G, M, or T
  switch (command_code) {
    case 'G': switch (codenum) {

      // G0, G1
      case 0:
      case 1:
        #if IS_SCARA
          gcode_G0_G1(codenum == 0);
        #else
          gcode_G0_G1();
        #endif
        break;

      // G2, G3
      #if ENABLED(ARC_SUPPORT) && DISABLED(SCARA)
        case 2: // G2  - CW ARC
        case 3: // G3  - CCW ARC
          gcode_G2_G3(codenum == 2);
          break;
      #endif

      // G4 Dwell
      case 4:
        gcode_G4();
        break;

      #if ENABLED(BEZIER_CURVE_SUPPORT)
        // G5
        case 5: // G5  - Cubic B_spline
          gcode_G5();
          break;
      #endif // BEZIER_CURVE_SUPPORT

      #if ENABLED(FWRETRACT)
        case 10: // G10: retract
        case 11: // G11: retract_recover
          gcode_G10_G11(codenum == 10);
          break;
      #endif // FWRETRACT
//&begin[Clean_Nozzle]
      #if ENABLED(NOZZLE_CLEAN_FEATURE)
        case 12:
          gcode_G12(); // G12: Nozzle Clean
          break;
      #endif // NOZZLE_CLEAN_FEATURE
//&end[Clean_Nozzle]
//&begin[Inch_Mode_Support]
      #if ENABLED(INCH_MODE_SUPPORT)
        case 20: //G20: Inch Mode
          gcode_G20();
          break;

        case 21: //G21: MM Mode
          gcode_G21();
          break;
      #endif // INCH_MODE_SUPPORT
//&end[Inch_Mode_Support]
	//&begin[Park_Nozzle]
      #if ENABLED(NOZZLE_PARK_FEATURE)
        case 27: // G27: Nozzle Park
          gcode_G27();
          break;
      #endif // NOZZLE_PARK_FEATURE
	//&end[Park_Nozzle]
	  
      case 28: // G28: Home all axes, one at a time
        gcode_G28();
        break;

      #if PLANNER_LEVELING
        case 29: // G29 Detailed Z probe, probes the bed at 3 or more points.
          gcode_G29();
          break;
      #endif // PLANNER_LEVELING

      #if HAS_BED_PROBE

        case 30: // G30 Single Z probe
          gcode_G30();
          break;

        #if ENABLED(Z_PROBE_SLED)

            case 31: // G31: dock the sled
              gcode_G31();
              break;

            case 32: // G32: undock the sled
              gcode_G32();
              break;

        #endif // Z_PROBE_SLED
      #endif // HAS_BED_PROBE

      #if ENABLED(G38_PROBE_TARGET)
        case 38: // G38.2 & G38.3
          if (subcode == 2 || subcode == 3)
            gcode_G38(subcode == 2);
          break;
      #endif

      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;

      case 92: // G92
        gcode_G92();
        break;
    }
    break;

    case 'M': switch (codenum) {
		//&begin[Emergency_Command_Parser]
      #if ENABLED(ULTIPANEL) || ENABLED(EMERGENCY_PARSER)
        case 0: // M0: Unconditional stop - Wait for user button press on LCD
        case 1: // M1: Conditional stop - Wait for user button press on LCD
          gcode_M0_M1();
          break;
      #endif // ULTIPANEL
//&end[Emergency_Command_Parser]
      case 17: // M17: Enable all stepper motors
        gcode_M17();
        break;

      #if ENABLED(SDSUPPORT)
        case 20: // M20: list SD card
          gcode_M20(); break;
        case 21: // M21: init SD card
          gcode_M21(); break;
        case 22: // M22: release SD card
          gcode_M22(); break;
        case 23: // M23: Select file
          gcode_M23(); break;
        case 24: // M24: Start SD print
          gcode_M24(); break;
        case 25: // M25: Pause SD print
          gcode_M25(); break;
        case 26: // M26: Set SD index
          gcode_M26(); break;
        case 27: // M27: Get SD status
          gcode_M27(); break;
        case 28: // M28: Start SD write
          gcode_M28(); break;
        case 29: // M29: Stop SD write
          gcode_M29(); break;
        case 30: // M30 <filename> Delete File
          gcode_M30(); break;
        case 32: // M32: Select file and start SD print
          gcode_M32(); break;

        #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
          case 33: // M33: Get the long full path to a file or folder
            gcode_M33(); break;
        #endif

        case 928: // M928: Start SD write
          gcode_M928(); break;
      #endif //SDSUPPORT

      case 31: // M31: Report time since the start of SD print or last M109
        gcode_M31(); break;

      case 42: // M42: Change pin state
        gcode_M42(); break;
//&begin[PINS_DEBUGGING]
      #if ENABLED(PINS_DEBUGGING)
        case 43: // M43: Read pin state
          gcode_M43(); break;
      #endif
//&end[PINS_DEBUGGING]
      #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
        case 48: // M48: Z probe repeatability test
          gcode_M48();
          break;
      #endif // Z_MIN_PROBE_REPEATABILITY_TEST

      case 75: // M75: Start print timer
        gcode_M75(); break;
      case 76: // M76: Pause print timer
        gcode_M76(); break;
      case 77: // M77: Stop print timer
        gcode_M77(); break;

      #if ENABLED(PRINTCOUNTER)
        case 78: // M78: Show print statistics
          gcode_M78(); break;
      #endif

      #if ENABLED(M100_FREE_MEMORY_WATCHER)
        case 100: // M100: Free Memory Report
          gcode_M100();
          break;
      #endif

      case 104: // M104: Set hot end temperature
        gcode_M104();
        break;

      case 110: // M110: Set Current Line Number
        gcode_M110();
        break;

      case 111: // M111: Set debug level
        gcode_M111();
        break;
//&begin[Emergency_Command_Parser]
      #if DISABLED(EMERGENCY_PARSER)

        case 108: // M108: Cancel Waiting
          gcode_M108();
          break;

        case 112: // M112: Emergency Stop
          gcode_M112();
          break;

        case 410: // M410 quickstop - Abort all the planned moves.
          gcode_M410();
          break;

      #endif
//&end[Emergency_Command_Parser]
      #if ENABLED(HOST_KEEPALIVE_FEATURE)
        case 113: // M113: Set Host Keepalive interval
          gcode_M113();
          break;
      #endif

      case 140: // M140: Set bed temperature
        gcode_M140();
        break;

      case 105: // M105: Report current temperature
        gcode_M105();
        KEEPALIVE_STATE(NOT_BUSY);
        return; // "ok" already printed

      #if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
        case 155: // M155: Set temperature auto-report interval
          gcode_M155();
          break;
      #endif

      case 109: // M109: Wait for hotend temperature to reach target
        gcode_M109();
        break;

      #if HAS_TEMP_BED
        case 190: // M190: Wait for bed temperature to reach target
          gcode_M190();
          break;
      #endif // HAS_TEMP_BED

      #if FAN_COUNT > 0
        case 106: // M106: Fan On
          gcode_M106();
          break;
        case 107: // M107: Fan Off
          gcode_M107();
          break;
      #endif // FAN_COUNT > 0

      #if ENABLED(BARICUDA)
        // PWM for HEATER_1_PIN
        #if HAS_HEATER_1
          case 126: // M126: valve open
            gcode_M126();
            break;
          case 127: // M127: valve closed
            gcode_M127();
            break;
        #endif // HAS_HEATER_1

        // PWM for HEATER_2_PIN
        #if HAS_HEATER_2
          case 128: // M128: valve open
            gcode_M128();
            break;
          case 129: // M129: valve closed
            gcode_M129();
            break;
        #endif // HAS_HEATER_2
      #endif // BARICUDA

      #if HAS_POWER_SWITCH

        case 80: // M80: Turn on Power Supply
          gcode_M80();
          break;

      #endif // HAS_POWER_SWITCH

      case 81: // M81: Turn off Power, including Power Supply, if possible
        gcode_M81();
        break;

      case 82: // M83: Set E axis normal mode (same as other axes)
        gcode_M82();
        break;
      case 83: // M83: Set E axis relative mode
        gcode_M83();
        break;
      case 18: // M18 => M84
      case 84: // M84: Disable all steppers or set timeout
        gcode_M18_M84();
        break;
      case 85: // M85: Set inactivity stepper shutdown timeout
        gcode_M85();
        break;
      case 92: // M92: Set the steps-per-unit for one or more axes
        gcode_M92();
        break;
      case 114: // M114: Report current position
        gcode_M114();
        break;
      case 115: // M115: Report capabilities
        gcode_M115();
        break;
      case 117: // M117: Set LCD message text, if possible
        gcode_M117();
        break;
      case 119: // M119: Report endstop states
        gcode_M119();
        break;
      case 120: // M120: Enable endstops
        gcode_M120();
        break;
      case 121: // M121: Disable endstops
        gcode_M121();
        break;

      #if ENABLED(HAVE_TMC2130DRIVER)
        case 122: // M122: Diagnose, used to debug TMC2130
          gcode_M122();
          break;
      #endif

      #if ENABLED(ULTIPANEL)

        case 145: // M145: Set material heatup parameters
          gcode_M145();
          break;

      #endif

      #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
        case 149: // M149: Set temperature units
          gcode_M149();
          break;
      #endif

      #if ENABLED(BLINKM) || ENABLED(RGB_LED)

        case 150: // M150: Set Status LED Color
          gcode_M150();
          break;

      #endif // BLINKM

      #if ENABLED(MIXING_EXTRUDER)
        case 163: // M163: Set a component weight for mixing extruder
          gcode_M163();
          break;
        #if MIXING_VIRTUAL_TOOLS > 1
          case 164: // M164: Save current mix as a virtual extruder
            gcode_M164();
            break;
        #endif
        #if ENABLED(DIRECT_MIXING_IN_G1)
          case 165: // M165: Set multiple mix weights
            gcode_M165();
            break;
        #endif
      #endif

      case 200: // M200: Set filament diameter, E to cubic units
        gcode_M200();
        break;
      case 201: // M201: Set max acceleration for print moves (units/s^2)
        gcode_M201();
        break;
      #if 0 // Not used for Sprinter/grbl gen6
        case 202: // M202
          gcode_M202();
          break;
      #endif
      case 203: // M203: Set max feedrate (units/sec)
        gcode_M203();
        break;
      case 204: // M204: Set acceleration
        gcode_M204();
        break;
      case 205: //M205: Set advanced settings
        gcode_M205();
        break;
      case 206: // M206: Set home offsets
        gcode_M206();
        break;

      #if ENABLED(DELTA)
        case 665: // M665: Set delta configurations
          gcode_M665();
          break;
      #endif

      #if ENABLED(DELTA) || ENABLED(Z_DUAL_ENDSTOPS)
        case 666: // M666: Set delta or dual endstop adjustment
          gcode_M666();
          break;
      #endif

      #if ENABLED(FWRETRACT)
        case 207: // M207: Set Retract Length, Feedrate, and Z lift
          gcode_M207();
          break;
        case 208: // M208: Set Recover (unretract) Additional Length and Feedrate
          gcode_M208();
          break;
        case 209: // M209: Turn Automatic Retract Detection on/off
          gcode_M209();
          break;
      #endif // FWRETRACT

      case 211: // M211: Enable, Disable, and/or Report software endstops
        gcode_M211();
        break;

      #if HOTENDS > 1
        case 218: // M218: Set a tool offset
          gcode_M218();
          break;
      #endif

      case 220: // M220: Set Feedrate Percentage: S<percent> ("FR" on your LCD)
        gcode_M220();
        break;

      case 221: // M221: Set Flow Percentage
        gcode_M221();
        break;

      case 226: // M226: Wait until a pin reaches a state
        gcode_M226();
        break;

      #if HAS_SERVOS
        case 280: // M280: Set servo position absolute
          gcode_M280();
          break;
      #endif // HAS_SERVOS

      #if HAS_BUZZER
        case 300: // M300: Play beep tone
          gcode_M300();
          break;
      #endif // HAS_BUZZER

      #if ENABLED(PIDTEMP)
        case 301: // M301: Set hotend PID parameters
          gcode_M301();
          break;
      #endif // PIDTEMP

      #if ENABLED(PIDTEMPBED)
        case 304: // M304: Set bed PID parameters
          gcode_M304();
          break;
      #endif // PIDTEMPBED

      #if defined(CHDK) || HAS_PHOTOGRAPH
        case 240: // M240: Trigger a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
          gcode_M240();
          break;
      #endif // CHDK || PHOTOGRAPH_PIN

      #if HAS_LCD_CONTRAST
        case 250: // M250: Set LCD contrast
          gcode_M250();
          break;
      #endif // HAS_LCD_CONTRAST

      #if ENABLED(EXPERIMENTAL_I2CBUS)

        case 260: // M260: Send data to an i2c slave
          gcode_M260();
          break;

        case 261: // M261: Request data from an i2c slave
          gcode_M261();
          break;

      #endif // EXPERIMENTAL_I2CBUS

      #if ENABLED(PREVENT_COLD_EXTRUSION)
        case 302: // M302: Allow cold extrudes (set the minimum extrude temperature)
          gcode_M302();
          break;
      #endif // PREVENT_COLD_EXTRUSION

      case 303: // M303: PID autotune
        gcode_M303();
        break;

      #if ENABLED(MORGAN_SCARA)
        case 360:  // M360: SCARA Theta pos1
          if (gcode_M360()) return;
          break;
        case 361:  // M361: SCARA Theta pos2
          if (gcode_M361()) return;
          break;
        case 362:  // M362: SCARA Psi pos1
          if (gcode_M362()) return;
          break;
        case 363:  // M363: SCARA Psi pos2
          if (gcode_M363()) return;
          break;
        case 364:  // M364: SCARA Psi pos3 (90 deg to Theta)
          if (gcode_M364()) return;
          break;
      #endif // SCARA

      case 400: // M400: Finish all moves
        gcode_M400();
        break;

      #if HAS_BED_PROBE
        case 401: // M401: Deploy probe
          gcode_M401();
          break;
        case 402: // M402: Stow probe
          gcode_M402();
          break;
      #endif // HAS_BED_PROBE

      #if ENABLED(FILAMENT_WIDTH_SENSOR)
        case 404:  // M404: Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width
          gcode_M404();
          break;
        case 405:  // M405: Turn on filament sensor for control
          gcode_M405();
          break;
        case 406:  // M406: Turn off filament sensor for control
          gcode_M406();
          break;
        case 407:   // M407: Display measured filament diameter
          gcode_M407();
          break;
      #endif // ENABLED(FILAMENT_WIDTH_SENSOR)

      #if PLANNER_LEVELING
        case 420: // M420: Enable/Disable Bed Leveling
          gcode_M420();
          break;
      #endif

      #if ENABLED(MESH_BED_LEVELING)
        case 421: // M421: Set a Mesh Bed Leveling Z coordinate
          gcode_M421();
          break;
      #endif

      case 428: // M428: Apply current_position to home_offset
        gcode_M428();
        break;

      case 500: // M500: Store settings in EEPROM
        gcode_M500();
        break;
      case 501: // M501: Read settings from EEPROM
        gcode_M501();
        break;
      case 502: // M502: Revert to default settings
        gcode_M502();
        break;
      case 503: // M503: print settings currently in memory
        gcode_M503();
        break;

      #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
        case 540: // M540: Set abort on endstop hit for SD printing
          gcode_M540();
          break;
      #endif

      #if HAS_BED_PROBE
        case 851: // M851: Set Z Probe Z Offset
          gcode_M851();
          break;
      #endif // HAS_BED_PROBE
//&begin[FILAMENT_CHANGE_FEATURE]
      #if ENABLED(FILAMENT_CHANGE_FEATURE)
        case 600: // M600: Pause for filament change
          gcode_M600();
          break;
      #endif // FILAMENT_CHANGE_FEATURE
//&end[FILAMENT_CHANGE_FEATURE]
      #if ENABLED(DUAL_X_CARRIAGE)
        case 605: // M605: Set Dual X Carriage movement mode
          gcode_M605();
          break;
      #endif // DUAL_X_CARRIAGE
//&begin[LIN_ADVANCE]
      #if ENABLED(LIN_ADVANCE)
        case 905: // M905: Set advance K factor.
          gcode_M905();
          break;
      #endif
//&end[LIN_ADVANCE]
      case 907: // M907: Set digital trimpot motor current using axis codes.
        gcode_M907();
        break;

      #if HAS_DIGIPOTSS || ENABLED(DAC_STEPPER_CURRENT)

        case 908: // M908: Control digital trimpot directly.
          gcode_M908();
          break;

        #if ENABLED(DAC_STEPPER_CURRENT) // As with Printrbot RevF

          case 909: // M909: Print digipot/DAC current value
            gcode_M909();
            break;

          case 910: // M910: Commit digipot/DAC value to external EEPROM
            gcode_M910();
            break;

        #endif

      #endif // HAS_DIGIPOTSS || DAC_STEPPER_CURRENT

      #if HAS_MICROSTEPS

        case 350: // M350: Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
          gcode_M350();
          break;

        case 351: // M351: Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
          gcode_M351();
          break;

      #endif // HAS_MICROSTEPS
//&begin[CASE_LIGHT]
      #if HAS_CASE_LIGHT

        case 355: // M355 Turn case lights on/off
          gcode_M355();
          break;

      #endif // HAS_CASE_LIGHT
//&end[CASE_LIGHT]
      case 999: // M999: Restart after being Stopped
        gcode_M999();
        break;
    }
    break;

    case 'T':
      gcode_T(codenum);
      break;

    default: code_is_good = false;
  }

  KEEPALIVE_STATE(NOT_BUSY);

ExitUnknownCommand:

  // Still unknown command? Throw an error
  if (!code_is_good) unknown_command_error();

  ok_to_send();
}

/**
 * Send a "Resend: nnn" message to the host to
 * indicate that a command needs to be re-sent.
 */
void FlushSerialRequestResend() {
  //char command_queue[cmd_queue_index_r][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ok_to_send();
}

/**
 * Send an "ok" message to the host, indicating
 * that a command was successfully processed.
 *
 * If ADVANCED_OK is enabled also include:
 *   N<int>  Line number of the command, if any
 *   P<int>  Planner space remaining
 *   B<int>  Block queue space remaining
 */
void ok_to_send() {
  refresh_cmd_timeout();
  if (!send_ok[cmd_queue_index_r]) return;
  SERIAL_PROTOCOLPGM(MSG_OK);
  #if ENABLED(ADVANCED_OK)
    char* p = command_queue[cmd_queue_index_r];
    if (*p == 'N') {
      SERIAL_PROTOCOL(' ');
      SERIAL_ECHO(*p++);
      while (NUMERIC_SIGNED(*p))
        SERIAL_ECHO(*p++);
    }
    SERIAL_PROTOCOLPGM(" P"); SERIAL_PROTOCOL(int(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
    SERIAL_PROTOCOLPGM(" B"); SERIAL_PROTOCOL(BUFSIZE - commands_in_queue);
  #endif
  SERIAL_EOL;
}
//&end[Command_Input_Process]

//&begin[Endstop]
#if ENABLED(min_software_endstops) || ENABLED(max_software_endstops)

  /**
   * Constrain the given coordinates to the software endstops.
   */
  void clamp_to_software_endstops(float target[XYZ]) {
    #if ENABLED(min_software_endstops)
      NOLESS(target[X_AXIS], soft_endstop_min[X_AXIS]);
      NOLESS(target[Y_AXIS], soft_endstop_min[Y_AXIS]);
      NOLESS(target[Z_AXIS], soft_endstop_min[Z_AXIS]);
    #endif
    #if ENABLED(max_software_endstops)
      NOMORE(target[X_AXIS], soft_endstop_max[X_AXIS]);
      NOMORE(target[Y_AXIS], soft_endstop_max[Y_AXIS]);
      NOMORE(target[Z_AXIS], soft_endstop_max[Z_AXIS]);
    #endif
  }

#endif
//&end[Endstop]

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

  #if ENABLED(ABL_BILINEAR_SUBDIVISION)
    #define ABL_BG_SPACING(A) bilinear_grid_spacing_virt[A]
    #define ABL_BG_POINTS_X   ABL_GRID_POINTS_VIRT_X
    #define ABL_BG_POINTS_Y   ABL_GRID_POINTS_VIRT_Y
    #define ABL_BG_GRID(X,Y)  bed_level_grid_virt[X][Y]
  #else
    #define ABL_BG_SPACING(A) bilinear_grid_spacing[A]
    #define ABL_BG_POINTS_X   ABL_GRID_POINTS_X
    #define ABL_BG_POINTS_Y   ABL_GRID_POINTS_Y
    #define ABL_BG_GRID(X,Y)  bed_level_grid[X][Y]
  #endif

  // Get the Z adjustment for non-linear bed leveling
  float bilinear_z_offset(float cartesian[XYZ]) {

    // XY relative to the probed area
    const float x = RAW_X_POSITION(cartesian[X_AXIS]) - bilinear_start[X_AXIS],
                y = RAW_Y_POSITION(cartesian[Y_AXIS]) - bilinear_start[Y_AXIS];

    // Convert to grid box units
    float ratio_x = x / ABL_BG_SPACING(X_AXIS),
          ratio_y = y / ABL_BG_SPACING(Y_AXIS);

    // Whole units for the grid line indices. Constrained within bounds.
    const int gridx = constrain(floor(ratio_x), 0, ABL_BG_POINTS_X - 1),
              gridy = constrain(floor(ratio_y), 0, ABL_BG_POINTS_Y - 1),
              nextx = min(gridx + 1, ABL_BG_POINTS_X - 1),
              nexty = min(gridy + 1, ABL_BG_POINTS_Y - 1);

    // Subtract whole to get the ratio within the grid box
    ratio_x -= gridx; ratio_y -= gridy;

    // Never less than 0.0. (Over 1.0 is fine due to previous contraints.)
    NOLESS(ratio_x, 0); NOLESS(ratio_y, 0);

    // Z at the box corners
    const float z1 = ABL_BG_GRID(gridx, gridy),  // left-front
                z2 = ABL_BG_GRID(gridx, nexty),  // left-back
                z3 = ABL_BG_GRID(nextx, gridy),  // right-front
                z4 = ABL_BG_GRID(nextx, nexty),  // right-back

                // Bilinear interpolate
                L = z1 + (z2 - z1) * ratio_y,   // Linear interp. LF -> LB
                R = z3 + (z4 - z3) * ratio_y,   // Linear interp. RF -> RB
                offset = L + ratio_x * (R - L);

    /*
    static float last_offset = 0;
    if (fabs(last_offset - offset) > 0.2) {
      SERIAL_ECHOPGM("Sudden Shift at ");
      SERIAL_ECHOPAIR("x=", x);
      SERIAL_ECHOPAIR(" / ", bilinear_grid_spacing[X_AXIS]);
      SERIAL_ECHOLNPAIR(" -> gridx=", gridx);
      SERIAL_ECHOPAIR(" y=", y);
      SERIAL_ECHOPAIR(" / ", bilinear_grid_spacing[Y_AXIS]);
      SERIAL_ECHOLNPAIR(" -> gridy=", gridy);
      SERIAL_ECHOPAIR(" ratio_x=", ratio_x);
      SERIAL_ECHOLNPAIR(" ratio_y=", ratio_y);
      SERIAL_ECHOPAIR(" z1=", z1);
      SERIAL_ECHOPAIR(" z2=", z2);
      SERIAL_ECHOPAIR(" z3=", z3);
      SERIAL_ECHOLNPAIR(" z4=", z4);
      SERIAL_ECHOPAIR(" L=", L);
      SERIAL_ECHOPAIR(" R=", R);
      SERIAL_ECHOLNPAIR(" offset=", offset);
    }
    last_offset = offset;
    //*/

    return offset;
  }

#endif // AUTO_BED_LEVELING_BILINEAR

#if ENABLED(DELTA)

  /**
   * Recalculate factors used for delta kinematics whenever
   * settings have been changed (e.g., by M665).
   */
  void recalc_delta_settings(float radius, float diagonal_rod) {
    delta_tower1_x = -SIN_60 * (radius + DELTA_RADIUS_TRIM_TOWER_1);  // front left tower
    delta_tower1_y = -COS_60 * (radius + DELTA_RADIUS_TRIM_TOWER_1);
    delta_tower2_x =  SIN_60 * (radius + DELTA_RADIUS_TRIM_TOWER_2);  // front right tower
    delta_tower2_y = -COS_60 * (radius + DELTA_RADIUS_TRIM_TOWER_2);
    delta_tower3_x = 0.0;                                             // back middle tower
    delta_tower3_y = (radius + DELTA_RADIUS_TRIM_TOWER_3);
    delta_diagonal_rod_2_tower_1 = sq(diagonal_rod + delta_diagonal_rod_trim_tower_1);
    delta_diagonal_rod_2_tower_2 = sq(diagonal_rod + delta_diagonal_rod_trim_tower_2);
    delta_diagonal_rod_2_tower_3 = sq(diagonal_rod + delta_diagonal_rod_trim_tower_3);
  }

  #if ENABLED(DELTA_FAST_SQRT)
    /**
     * Fast inverse sqrt from Quake III Arena
     * See: https://en.wikipedia.org/wiki/Fast_inverse_square_root
     */
    float Q_rsqrt(float number) {
      long i;
      float x2, y;
      const float threehalfs = 1.5f;
      x2 = number * 0.5f;
      y  = number;
      i  = * ( long * ) &y;                       // evil floating point bit level hacking
      i  = 0x5f3759df - ( i >> 1 );               // what the f***?
      y  = * ( float * ) &i;
      y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
      // y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed
      return y;
    }

    #define _SQRT(n) (1.0f / Q_rsqrt(n))

  #else

    #define _SQRT(n) sqrt(n)

  #endif

  /**
   * Delta Inverse Kinematics
   *
   * Calculate the tower positions for a given logical
   * position, storing the result in the delta[] array.
   *
   * This is an expensive calculation, requiring 3 square
   * roots per segmented linear move, and strains the limits
   * of a Mega2560 with a Graphical Display.
   *
   * Suggested optimizations include:
   *
   * - Disable the home_offset (M206) and/or position_shift (G92)
   *   features to remove up to 12 float additions.
   *
   * - Use a fast-inverse-sqrt function and add the reciprocal.
   *   (see above)
   */

  // Macro to obtain the Z position of an individual tower
  #define DELTA_Z(T) raw[Z_AXIS] + _SQRT(    \
    delta_diagonal_rod_2_tower_##T - HYPOT2( \
        delta_tower##T##_x - raw[X_AXIS],    \
        delta_tower##T##_y - raw[Y_AXIS]     \
      )                                      \
    )

  #define DELTA_RAW_IK() do {   \
    delta[A_AXIS] = DELTA_Z(1); \
    delta[B_AXIS] = DELTA_Z(2); \
    delta[C_AXIS] = DELTA_Z(3); \
  } while(0)

  #define DELTA_LOGICAL_IK() do {      \
    const float raw[XYZ] = {           \
      RAW_X_POSITION(logical[X_AXIS]), \
      RAW_Y_POSITION(logical[Y_AXIS]), \
      RAW_Z_POSITION(logical[Z_AXIS])  \
    };                                 \
    DELTA_RAW_IK();                    \
  } while(0)

  #define DELTA_DEBUG() do { \
      SERIAL_ECHOPAIR("cartesian X:", raw[X_AXIS]); \
      SERIAL_ECHOPAIR(" Y:", raw[Y_AXIS]);          \
      SERIAL_ECHOLNPAIR(" Z:", raw[Z_AXIS]);        \
      SERIAL_ECHOPAIR("delta A:", delta[A_AXIS]);   \
      SERIAL_ECHOPAIR(" B:", delta[B_AXIS]);        \
      SERIAL_ECHOLNPAIR(" C:", delta[C_AXIS]);      \
    } while(0)

  void inverse_kinematics(const float logical[XYZ]) {
    DELTA_LOGICAL_IK();
    // DELTA_DEBUG();
  }

  /**
   * Calculate the highest Z position where the
   * effector has the full range of XY motion.
   */
  float delta_safe_distance_from_top() {
    float cartesian[XYZ] = {
      LOGICAL_X_POSITION(0),
      LOGICAL_Y_POSITION(0),
      LOGICAL_Z_POSITION(0)
    };
    inverse_kinematics(cartesian);
    float distance = delta[A_AXIS];
    cartesian[Y_AXIS] = LOGICAL_Y_POSITION(DELTA_PRINTABLE_RADIUS);
    inverse_kinematics(cartesian);
    return abs(distance - delta[A_AXIS]);
  }

  /**
   * Delta Forward Kinematics
   *
   * See the Wikipedia article "Trilateration"
   * https://en.wikipedia.org/wiki/Trilateration
   *
   * Establish a new coordinate system in the plane of the
   * three carriage points. This system has its origin at
   * tower1, with tower2 on the X axis. Tower3 is in the X-Y
   * plane with a Z component of zero.
   * We will define unit vectors in this coordinate system
   * in our original coordinate system. Then when we calculate
   * the Xnew, Ynew and Znew values, we can translate back into
   * the original system by moving along those unit vectors
   * by the corresponding values.
   *
   * Variable names matched to Marlin, c-version, and avoid the
   * use of any vector library.
   *
   * by Andreas Hardtung 2016-06-07
   * based on a Java function from "Delta Robot Kinematics V3"
   * by Steve Graves
   *
   * The result is stored in the cartes[] array.
   */
  void forward_kinematics_DELTA(float z1, float z2, float z3) {
    // Create a vector in old coordinates along x axis of new coordinate
    float p12[3] = { delta_tower2_x - delta_tower1_x, delta_tower2_y - delta_tower1_y, z2 - z1 };

    // Get the Magnitude of vector.
    float d = sqrt( sq(p12[0]) + sq(p12[1]) + sq(p12[2]) );

    // Create unit vector by dividing by magnitude.
    float ex[3] = { p12[0] / d, p12[1] / d, p12[2] / d };

    // Get the vector from the origin of the new system to the third point.
    float p13[3] = { delta_tower3_x - delta_tower1_x, delta_tower3_y - delta_tower1_y, z3 - z1 };

    // Use the dot product to find the component of this vector on the X axis.
    float i = ex[0] * p13[0] + ex[1] * p13[1] + ex[2] * p13[2];

    // Create a vector along the x axis that represents the x component of p13.
    float iex[3] = { ex[0] * i, ex[1] * i, ex[2] * i };

    // Subtract the X component from the original vector leaving only Y. We use the
    // variable that will be the unit vector after we scale it.
    float ey[3] = { p13[0] - iex[0], p13[1] - iex[1], p13[2] - iex[2] };

    // The magnitude of Y component
    float j = sqrt( sq(ey[0]) + sq(ey[1]) + sq(ey[2]) );

    // Convert to a unit vector
    ey[0] /= j; ey[1] /= j;  ey[2] /= j;

    // The cross product of the unit x and y is the unit z
    // float[] ez = vectorCrossProd(ex, ey);
    float ez[3] = {
      ex[1] * ey[2] - ex[2] * ey[1],
      ex[2] * ey[0] - ex[0] * ey[2],
      ex[0] * ey[1] - ex[1] * ey[0]
    };

    // We now have the d, i and j values defined in Wikipedia.
    // Plug them into the equations defined in Wikipedia for Xnew, Ynew and Znew
    float Xnew = (delta_diagonal_rod_2_tower_1 - delta_diagonal_rod_2_tower_2 + sq(d)) / (d * 2),
          Ynew = ((delta_diagonal_rod_2_tower_1 - delta_diagonal_rod_2_tower_3 + HYPOT2(i, j)) / 2 - i * Xnew) / j,
          Znew = sqrt(delta_diagonal_rod_2_tower_1 - HYPOT2(Xnew, Ynew));

    // Start from the origin of the old coordinates and add vectors in the
    // old coords that represent the Xnew, Ynew and Znew to find the point
    // in the old system.
    cartes[X_AXIS] = delta_tower1_x + ex[0] * Xnew + ey[0] * Ynew - ez[0] * Znew;
    cartes[Y_AXIS] = delta_tower1_y + ex[1] * Xnew + ey[1] * Ynew - ez[1] * Znew;
    cartes[Z_AXIS] =             z1 + ex[2] * Xnew + ey[2] * Ynew - ez[2] * Znew;
  }

  void forward_kinematics_DELTA(float point[ABC]) {
    forward_kinematics_DELTA(point[A_AXIS], point[B_AXIS], point[C_AXIS]);
  }

#endif // DELTA

//&begin[Moter_Type_Stepper]
/**
 * Get the stepper positions in the cartes[] array.
 * Forward kinematics are applied for DELTA and SCARA.
 *
 * The result is in the current coordinate space with
 * leveling applied. The coordinates need to be run through
 * unapply_leveling to obtain the "ideal" coordinates
 * suitable for current_position, etc.
 */
void get_cartesian_from_steppers() {
  #if ENABLED(DELTA)
    forward_kinematics_DELTA(
      stepper.get_axis_position_mm(A_AXIS),
      stepper.get_axis_position_mm(B_AXIS),
      stepper.get_axis_position_mm(C_AXIS)
    );
    cartes[X_AXIS] += LOGICAL_X_POSITION(0);
    cartes[Y_AXIS] += LOGICAL_Y_POSITION(0);
    cartes[Z_AXIS] += LOGICAL_Z_POSITION(0);
  #elif IS_SCARA
    forward_kinematics_SCARA(
      stepper.get_axis_position_degrees(A_AXIS),
      stepper.get_axis_position_degrees(B_AXIS)
    );
    cartes[X_AXIS] += LOGICAL_X_POSITION(0);
    cartes[Y_AXIS] += LOGICAL_Y_POSITION(0);
    cartes[Z_AXIS] = stepper.get_axis_position_mm(Z_AXIS);
  #else
    cartes[X_AXIS] = stepper.get_axis_position_mm(X_AXIS);
    cartes[Y_AXIS] = stepper.get_axis_position_mm(Y_AXIS);
    cartes[Z_AXIS] = stepper.get_axis_position_mm(Z_AXIS);
  #endif
}

/**
 * Set the current_position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 */
void set_current_from_steppers_for_axis(const AxisEnum axis) {
  get_cartesian_from_steppers();
  #if PLANNER_LEVELING
    planner.unapply_leveling(cartes);
  #endif
  if (axis == ALL_AXES)
    memcpy(current_position, cartes, sizeof(cartes));
  else
    current_position[axis] = cartes[axis];
}
//&end[Moter_Type_Stepper]

#if ENABLED(MESH_BED_LEVELING)

  /**
   * Prepare a mesh-leveled linear move in a Cartesian setup,
   * splitting the move where it crosses mesh borders.
   */
  void mesh_line_to_destination(float fr_mm_s, uint8_t x_splits = 0xff, uint8_t y_splits = 0xff) {
    int cx1 = mbl.cell_index_x(RAW_CURRENT_POSITION(X_AXIS)),
        cy1 = mbl.cell_index_y(RAW_CURRENT_POSITION(Y_AXIS)),
        cx2 = mbl.cell_index_x(RAW_X_POSITION(destination[X_AXIS])),
        cy2 = mbl.cell_index_y(RAW_Y_POSITION(destination[Y_AXIS]));
    NOMORE(cx1, MESH_NUM_X_POINTS - 2);
    NOMORE(cy1, MESH_NUM_Y_POINTS - 2);
    NOMORE(cx2, MESH_NUM_X_POINTS - 2);
    NOMORE(cy2, MESH_NUM_Y_POINTS - 2);

    if (cx1 == cx2 && cy1 == cy2) {
      // Start and end on same mesh square
      line_to_destination(fr_mm_s);
      set_current_to_destination();
      return;
    }

    #define MBL_SEGMENT_END(A) (current_position[A ##_AXIS] + (destination[A ##_AXIS] - current_position[A ##_AXIS]) * normalized_dist)

    float normalized_dist, end[XYZE];

    // Split at the left/front border of the right/top square
    int8_t gcx = max(cx1, cx2), gcy = max(cy1, cy2);
    if (cx2 != cx1 && TEST(x_splits, gcx)) {
      memcpy(end, destination, sizeof(end));
      destination[X_AXIS] = LOGICAL_X_POSITION(mbl.get_probe_x(gcx));
      normalized_dist = (destination[X_AXIS] - current_position[X_AXIS]) / (end[X_AXIS] - current_position[X_AXIS]);
      destination[Y_AXIS] = MBL_SEGMENT_END(Y);
      CBI(x_splits, gcx);
    }
    else if (cy2 != cy1 && TEST(y_splits, gcy)) {
      memcpy(end, destination, sizeof(end));
      destination[Y_AXIS] = LOGICAL_Y_POSITION(mbl.get_probe_y(gcy));
      normalized_dist = (destination[Y_AXIS] - current_position[Y_AXIS]) / (end[Y_AXIS] - current_position[Y_AXIS]);
      destination[X_AXIS] = MBL_SEGMENT_END(X);
      CBI(y_splits, gcy);
    }
    else {
      // Already split on a border
      line_to_destination(fr_mm_s);
      set_current_to_destination();
      return;
    }

    destination[Z_AXIS] = MBL_SEGMENT_END(Z);
    destination[E_AXIS] = MBL_SEGMENT_END(E);

    // Do the split and look for more borders
    mesh_line_to_destination(fr_mm_s, x_splits, y_splits);

    // Restore destination from stack
    memcpy(destination, end, sizeof(end));
    mesh_line_to_destination(fr_mm_s, x_splits, y_splits);
  }

#elif ENABLED(AUTO_BED_LEVELING_BILINEAR) && !IS_KINEMATIC

  #define CELL_INDEX(A,V) ((RAW_##A##_POSITION(V) - bilinear_start[A##_AXIS]) / ABL_BG_SPACING(A##_AXIS))

  /**
   * Prepare a bilinear-leveled linear move on Cartesian,
   * splitting the move where it crosses grid borders.
   */
  void bilinear_line_to_destination(float fr_mm_s, uint16_t x_splits = 0xFFFF, uint16_t y_splits = 0xFFFF) {
    int cx1 = CELL_INDEX(X, current_position[X_AXIS]),
        cy1 = CELL_INDEX(Y, current_position[Y_AXIS]),
        cx2 = CELL_INDEX(X, destination[X_AXIS]),
        cy2 = CELL_INDEX(Y, destination[Y_AXIS]);
    cx1 = constrain(cx1, 0, ABL_BG_POINTS_X - 2);
    cy1 = constrain(cy1, 0, ABL_BG_POINTS_Y - 2);
    cx2 = constrain(cx2, 0, ABL_BG_POINTS_X - 2);
    cy2 = constrain(cy2, 0, ABL_BG_POINTS_Y - 2);

    if (cx1 == cx2 && cy1 == cy2) {
      // Start and end on same mesh square
      line_to_destination(fr_mm_s);
      set_current_to_destination();
      return;
    }

    #define LINE_SEGMENT_END(A) (current_position[A ##_AXIS] + (destination[A ##_AXIS] - current_position[A ##_AXIS]) * normalized_dist)

    float normalized_dist, end[XYZE];

    // Split at the left/front border of the right/top square
    int8_t gcx = max(cx1, cx2), gcy = max(cy1, cy2);
    if (cx2 != cx1 && TEST(x_splits, gcx)) {
      memcpy(end, destination, sizeof(end));
      destination[X_AXIS] = LOGICAL_X_POSITION(bilinear_start[X_AXIS] + ABL_BG_SPACING(X_AXIS) * gcx);
      normalized_dist = (destination[X_AXIS] - current_position[X_AXIS]) / (end[X_AXIS] - current_position[X_AXIS]);
      destination[Y_AXIS] = LINE_SEGMENT_END(Y);
      CBI(x_splits, gcx);
    }
    else if (cy2 != cy1 && TEST(y_splits, gcy)) {
      memcpy(end, destination, sizeof(end));
      destination[Y_AXIS] = LOGICAL_Y_POSITION(bilinear_start[Y_AXIS] + ABL_BG_SPACING(Y_AXIS) * gcy);
      normalized_dist = (destination[Y_AXIS] - current_position[Y_AXIS]) / (end[Y_AXIS] - current_position[Y_AXIS]);
      destination[X_AXIS] = LINE_SEGMENT_END(X);
      CBI(y_splits, gcy);
    }
    else {
      // Already split on a border
      line_to_destination(fr_mm_s);
      set_current_to_destination();
      return;
    }

    destination[Z_AXIS] = LINE_SEGMENT_END(Z);
    destination[E_AXIS] = LINE_SEGMENT_END(E);

    // Do the split and look for more borders
    bilinear_line_to_destination(fr_mm_s, x_splits, y_splits);

    // Restore destination from stack
    memcpy(destination, end, sizeof(end));
    bilinear_line_to_destination(fr_mm_s, x_splits, y_splits);
  }

#endif // AUTO_BED_LEVELING_BILINEAR

#if IS_KINEMATIC

  /**
   * Prepare a linear move in a DELTA or SCARA setup.
   *
   * This calls planner.buffer_line several times, adding
   * small incremental moves for DELTA or SCARA.
   */
  inline bool prepare_kinematic_move_to(float ltarget[NUM_AXIS]) {

    // Get the top feedrate of the move in the XY plane
    float _feedrate_mm_s = MMS_SCALED(feedrate_mm_s);

    // If the move is only in Z/E don't split up the move
    if (ltarget[X_AXIS] == current_position[X_AXIS] && ltarget[Y_AXIS] == current_position[Y_AXIS]) {
      planner.buffer_line_kinematic(ltarget, _feedrate_mm_s, active_extruder);
      return true;
    }

    // Get the cartesian distances moved in XYZE
    float difference[NUM_AXIS];
    LOOP_XYZE(i) difference[i] = ltarget[i] - current_position[i];

    // Get the linear distance in XYZ
    float cartesian_mm = sqrt(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));

    // If the move is very short, check the E move distance
    if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = abs(difference[E_AXIS]);

    // No E move either? Game over.
    if (UNEAR_ZERO(cartesian_mm)) return false;

    // Minimum number of seconds to move the given distance
    float seconds = cartesian_mm / _feedrate_mm_s;

    // The number of segments-per-second times the duration
    // gives the number of segments
    uint16_t segments = delta_segments_per_second * seconds;

    // For SCARA minimum segment size is 0.5mm
    #if IS_SCARA
      NOMORE(segments, cartesian_mm * 2);
    #endif

    // At least one segment is required
    NOLESS(segments, 1);

    // The approximate length of each segment
    float segment_distance[XYZE] = {
            difference[X_AXIS] / segments,
            difference[Y_AXIS] / segments,
            difference[Z_AXIS] / segments,
            difference[E_AXIS] / segments
          };

    // SERIAL_ECHOPAIR("mm=", cartesian_mm);
    // SERIAL_ECHOPAIR(" seconds=", seconds);
    // SERIAL_ECHOLNPAIR(" segments=", segments);

    // Drop one segment so the last move is to the exact target.
    // If there's only 1 segment, loops will be skipped entirely.
    --segments;

    // Using "raw" coordinates saves 6 float subtractions
    // per segment, saving valuable CPU cycles

    #if ENABLED(USE_RAW_KINEMATICS)

      // Get the raw current position as starting point
      float raw[XYZE] = {
        RAW_CURRENT_POSITION(X_AXIS),
        RAW_CURRENT_POSITION(Y_AXIS),
        RAW_CURRENT_POSITION(Z_AXIS),
        current_position[E_AXIS]
      };

      #define DELTA_VAR raw

      // Delta can inline its kinematics
      #if ENABLED(DELTA)
        #define DELTA_IK() DELTA_RAW_IK()
      #else
        #define DELTA_IK() inverse_kinematics(raw)
      #endif

    #else

      // Get the logical current position as starting point
      float logical[XYZE];
      memcpy(logical, current_position, sizeof(logical));

      #define DELTA_VAR logical

      // Delta can inline its kinematics
      #if ENABLED(DELTA)
        #define DELTA_IK() DELTA_LOGICAL_IK()
      #else
        #define DELTA_IK() inverse_kinematics(logical)
      #endif

    #endif

    #if ENABLED(USE_DELTA_IK_INTERPOLATION)

      // Only interpolate XYZ. Advance E normally.
      #define DELTA_NEXT(ADDEND) LOOP_XYZ(i) DELTA_VAR[i] += ADDEND;

      // Get the starting delta if interpolation is possible
      if (segments >= 2) {
        DELTA_IK();
        ADJUST_DELTA(DELTA_VAR); // Adjust Z if bed leveling is enabled
      }

      // Loop using decrement
      for (uint16_t s = segments + 1; --s;) {
        // Are there at least 2 moves left?
        if (s >= 2) {
          // Save the previous delta for interpolation
          float prev_delta[ABC] = { delta[A_AXIS], delta[B_AXIS], delta[C_AXIS] };

          // Get the delta 2 segments ahead (rather than the next)
          DELTA_NEXT(segment_distance[i] + segment_distance[i]);

          // Advance E normally
          DELTA_VAR[E_AXIS] += segment_distance[E_AXIS];

          // Get the exact delta for the move after this
          DELTA_IK();
          ADJUST_DELTA(DELTA_VAR); // Adjust Z if bed leveling is enabled

          // Move to the interpolated delta position first
          planner.buffer_line(
            (prev_delta[A_AXIS] + delta[A_AXIS]) * 0.5,
            (prev_delta[B_AXIS] + delta[B_AXIS]) * 0.5,
            (prev_delta[C_AXIS] + delta[C_AXIS]) * 0.5,
            DELTA_VAR[E_AXIS], _feedrate_mm_s, active_extruder
          );

          // Advance E once more for the next move
          DELTA_VAR[E_AXIS] += segment_distance[E_AXIS];

          // Do an extra decrement of the loop
          --s;
        }
        else {
          // Get the last segment delta. (Used when segments is odd)
          DELTA_NEXT(segment_distance[i]);
          DELTA_VAR[E_AXIS] += segment_distance[E_AXIS];
          DELTA_IK();
          ADJUST_DELTA(DELTA_VAR); // Adjust Z if bed leveling is enabled
        }

        // Move to the non-interpolated position
        planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], DELTA_VAR[E_AXIS], _feedrate_mm_s, active_extruder);
      }

    #else

      #define DELTA_NEXT(ADDEND) LOOP_XYZE(i) DELTA_VAR[i] += ADDEND;

      // For non-interpolated delta calculate every segment
      for (uint16_t s = segments + 1; --s;) {
        DELTA_NEXT(segment_distance[i]);
        planner.buffer_line_kinematic(DELTA_VAR, _feedrate_mm_s, active_extruder);
      }

    #endif

    // Since segment_distance is only approximate,
    // the final move must be to the exact destination.
    planner.buffer_line_kinematic(ltarget, _feedrate_mm_s, active_extruder);
    return true;
  }

#else // !IS_KINEMATIC

  /**
   * Prepare a linear move in a Cartesian setup.
   * If Mesh Bed Leveling is enabled, perform a mesh move.
   */
  inline bool prepare_move_to_destination_cartesian() {
    // Do not use feedrate_percentage for E or Z only moves
    if (current_position[X_AXIS] == destination[X_AXIS] && current_position[Y_AXIS] == destination[Y_AXIS]) {
      line_to_destination();
    }
    else {
      #if ENABLED(MESH_BED_LEVELING)
        if (mbl.active()) {
          mesh_line_to_destination(MMS_SCALED(feedrate_mm_s));
          return false;
        }
        else
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (planner.abl_enabled) {
          bilinear_line_to_destination(MMS_SCALED(feedrate_mm_s));
          return false;
        }
        else
      #endif
          line_to_destination(MMS_SCALED(feedrate_mm_s));
    }
    return true;
  }

#endif // !IS_KINEMATIC

#if ENABLED(DUAL_X_CARRIAGE)

  /**
   * Prepare a linear move in a dual X axis setup
   */
  inline bool prepare_move_to_destination_dualx() {
    if (active_extruder_parked) {
      switch (dual_x_carriage_mode) {
        case DXC_FULL_CONTROL_MODE:
          break;
        case DXC_AUTO_PARK_MODE:
          if (current_position[E_AXIS] == destination[E_AXIS]) {
            // This is a travel move (with no extrusion)
            // Skip it, but keep track of the current position
            // (so it can be used as the start of the next non-travel move)
            if (delayed_move_time != 0xFFFFFFFFUL) {
              set_current_to_destination();
              NOLESS(raised_parked_position[Z_AXIS], destination[Z_AXIS]);
              delayed_move_time = millis();
              return false;
            }
          }
          // unpark extruder: 1) raise, 2) move into starting XY position, 3) lower
          for (uint8_t i = 0; i < 3; i++)
            planner.buffer_line(
              i == 0 ? raised_parked_position[X_AXIS] : current_position[X_AXIS],
              i == 0 ? raised_parked_position[Y_AXIS] : current_position[Y_AXIS],
              i == 2 ? current_position[Z_AXIS] : raised_parked_position[Z_AXIS],
              current_position[E_AXIS],
              i == 1 ? PLANNER_XY_FEEDRATE() : planner.max_feedrate_mm_s[Z_AXIS],
              active_extruder
            );
          delayed_move_time = 0;
          active_extruder_parked = false;
          break;
        case DXC_DUPLICATION_MODE:
          if (active_extruder == 0) {
            // move duplicate extruder into correct duplication position.
            planner.set_position_mm(
              LOGICAL_X_POSITION(inactive_extruder_x_pos),
              current_position[Y_AXIS],
              current_position[Z_AXIS],
              current_position[E_AXIS]
            );
            planner.buffer_line(
              current_position[X_AXIS] + duplicate_extruder_x_offset,
              current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],
              planner.max_feedrate_mm_s[X_AXIS], 1
            );
            SYNC_PLAN_POSITION_KINEMATIC();
            stepper.synchronize();
            extruder_duplication_enabled = true;
            active_extruder_parked = false;
          }
          break;
      }
    }
    return true;
  }

#endif // DUAL_X_CARRIAGE

//&begin[Move_To_Destination]
/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 */
void prepare_move_to_destination() {
  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();

  #if ENABLED(PREVENT_COLD_EXTRUSION)

    if (!DEBUGGING(DRYRUN)) {
      if (destination[E_AXIS] != current_position[E_AXIS]) {
        if (thermalManager.tooColdToExtrude(active_extruder)) {
          current_position[E_AXIS] = destination[E_AXIS]; // Behave as if the move really took place, but ignore E part
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
        }
        #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
          if (labs(destination[E_AXIS] - current_position[E_AXIS]) > EXTRUDE_MAXLENGTH) {
            current_position[E_AXIS] = destination[E_AXIS]; // Behave as if the move really took place, but ignore E part
            SERIAL_ECHO_START;
            SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
          }
        #endif
      }
    }

  #endif

  #if IS_KINEMATIC
    if (!prepare_kinematic_move_to(destination)) return;
  #else
    #if ENABLED(DUAL_X_CARRIAGE)
      if (!prepare_move_to_destination_dualx()) return;
    #endif
    if (!prepare_move_to_destination_cartesian()) return;
  #endif

  set_current_to_destination();
}
//&end[Move_To_Destination]

//&begin[Arc_Movement]
#if ENABLED(ARC_SUPPORT)
  /**
   * Plan an arc in 2 dimensions
   *
   * The arc is approximated by generating many small linear segments.
   * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
   * Arcs should only be made relatively large (over 5mm), as larger arcs with
   * larger segments will tend to be more efficient. Your slicer should have
   * options for G2/G3 arc generation. In future these options may be GCode tunable.
   */
  void plan_arc(
    float logical[NUM_AXIS], // Destination position
    float* offset,           // Center of rotation relative to current_position
    uint8_t clockwise        // Clockwise?
  ) {

    float radius = HYPOT(offset[X_AXIS], offset[Y_AXIS]),
          center_X = current_position[X_AXIS] + offset[X_AXIS],
          center_Y = current_position[Y_AXIS] + offset[Y_AXIS],
          linear_travel = logical[Z_AXIS] - current_position[Z_AXIS],
          extruder_travel = logical[E_AXIS] - current_position[E_AXIS],
          r_X = -offset[X_AXIS],  // Radius vector from center to current location
          r_Y = -offset[Y_AXIS],
          rt_X = logical[X_AXIS] - center_X,
          rt_Y = logical[Y_AXIS] - center_Y;

    // CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_X * rt_Y - r_Y * rt_X, r_X * rt_X + r_Y * rt_Y);
    if (angular_travel < 0) angular_travel += RADIANS(360);
    if (clockwise) angular_travel -= RADIANS(360);

    // Make a circle if the angular rotation is 0
    if (angular_travel == 0 && current_position[X_AXIS] == logical[X_AXIS] && current_position[Y_AXIS] == logical[Y_AXIS])
      angular_travel += RADIANS(360);

    float mm_of_travel = HYPOT(angular_travel * radius, fabs(linear_travel));
    if (mm_of_travel < 0.001) return;

    uint16_t segments = floor(mm_of_travel / (MM_PER_ARC_SEGMENT));
    if (segments == 0) segments = 1;

    /**
     * Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     * and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
     *     r_T = [cos(phi) -sin(phi);
     *            sin(phi)  cos(phi)] * r ;
     *
     * For arc generation, the center of the circle is the axis of rotation and the radius vector is
     * defined from the circle center to the initial position. Each line segment is formed by successive
     * vector rotations. This requires only two cos() and sin() computations to form the rotation
     * matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     * all double numbers are single precision on the Arduino. (True double precision will not have
     * round off issues for CNC applications.) Single precision error can accumulate to be greater than
     * tool precision in some cases. Therefore, arc path correction is implemented.
     *
     * Small angle approximation may be used to reduce computation overhead further. This approximation
     * holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
     * theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     * to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
     * numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     * issue for CNC machines with the single precision Arduino calculations.
     *
     * This approximation also allows plan_arc to immediately insert a line segment into the planner
     * without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     * a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead.
     * This is important when there are successive arc motions.
     */
    // Vector rotation matrix values
    float arc_target[XYZE],
          theta_per_segment = angular_travel / segments,
          linear_per_segment = linear_travel / segments,
          extruder_per_segment = extruder_travel / segments,
          sin_T = theta_per_segment,
          cos_T = 1 - 0.5 * sq(theta_per_segment); // Small angle approximation

    // Initialize the linear axis
    arc_target[Z_AXIS] = current_position[Z_AXIS];

    // Initialize the extruder axis
    arc_target[E_AXIS] = current_position[E_AXIS];

    float fr_mm_s = MMS_SCALED(feedrate_mm_s);

    millis_t next_idle_ms = millis() + 200UL;

    int8_t count = 0;
    for (uint16_t i = 1; i < segments; i++) { // Iterate (segments-1) times

      thermalManager.manage_heater();
      if (ELAPSED(millis(), next_idle_ms)) {
        next_idle_ms = millis() + 200UL;
        idle();
      }

      if (++count < N_ARC_CORRECTION) {
        // Apply vector rotation matrix to previous r_X / 1
        float r_new_Y = r_X * sin_T + r_Y * cos_T;
        r_X = r_X * cos_T - r_Y * sin_T;
        r_Y = r_new_Y;
      }
      else {
        // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
        // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
        // To reduce stuttering, the sin and cos could be computed at different times.
        // For now, compute both at the same time.
        float cos_Ti = cos(i * theta_per_segment),
              sin_Ti = sin(i * theta_per_segment);
        r_X = -offset[X_AXIS] * cos_Ti + offset[Y_AXIS] * sin_Ti;
        r_Y = -offset[X_AXIS] * sin_Ti - offset[Y_AXIS] * cos_Ti;
        count = 0;
      }

      // Update arc_target location
      arc_target[X_AXIS] = center_X + r_X;
      arc_target[Y_AXIS] = center_Y + r_Y;
      arc_target[Z_AXIS] += linear_per_segment;
      arc_target[E_AXIS] += extruder_per_segment;

      clamp_to_software_endstops(arc_target);

      planner.buffer_line_kinematic(arc_target, fr_mm_s, active_extruder);
    }

    // Ensure last segment arrives at target location.
    planner.buffer_line_kinematic(logical, fr_mm_s, active_extruder);

    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    set_current_to_destination();
  }
#endif

#if ENABLED(BEZIER_CURVE_SUPPORT)

  void plan_cubic_move(const float offset[4]) {
    cubic_b_spline(current_position, destination, offset, MMS_SCALED(feedrate_mm_s), active_extruder);

    // As far as the parser is concerned, the position is now == destination. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    set_current_to_destination();
  }

#endif // BEZIER_CURVE_SUPPORT
//&end[Arc_Movement]


#if HAS_CONTROLLERFAN

  void controllerFan() {
    static millis_t lastMotorOn = 0; // Last time a motor was turned on
    static millis_t nextMotorCheck = 0; // Last time the state was checked
    millis_t ms = millis();
    if (ELAPSED(ms, nextMotorCheck)) {
      nextMotorCheck = ms + 2500UL; // Not a time critical function, so only check every 2.5s
      if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON || thermalManager.soft_pwm_bed > 0
          || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
          #if E_STEPPERS > 1
            || E1_ENABLE_READ == E_ENABLE_ON
            #if HAS_X2_ENABLE
              || X2_ENABLE_READ == X_ENABLE_ON
            #endif
            #if E_STEPPERS > 2
              || E2_ENABLE_READ == E_ENABLE_ON
              #if E_STEPPERS > 3
                || E3_ENABLE_READ == E_ENABLE_ON
              #endif
            #endif
          #endif
      ) {
        lastMotorOn = ms; //... set time to NOW so the fan will turn on
      }

      // Fan off if no steppers have been enabled for CONTROLLERFAN_SECS seconds
      uint8_t speed = (!lastMotorOn || ELAPSED(ms, lastMotorOn + (CONTROLLERFAN_SECS) * 1000UL)) ? 0 : CONTROLLERFAN_SPEED;

      // allows digital or PWM fan output to be used (see M42 handling)
      digitalWrite(CONTROLLERFAN_PIN, speed);
      analogWrite(CONTROLLERFAN_PIN, speed);
    }
  }

#endif // HAS_CONTROLLERFAN

#if ENABLED(MORGAN_SCARA)

  /**
   * Morgan SCARA Forward Kinematics. Results in cartes[].
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void forward_kinematics_SCARA(const float &a, const float &b) {

    float a_sin = sin(RADIANS(a)) * L1,
          a_cos = cos(RADIANS(a)) * L1,
          b_sin = sin(RADIANS(b)) * L2,
          b_cos = cos(RADIANS(b)) * L2;

    cartes[X_AXIS] = a_cos + b_cos + SCARA_OFFSET_X;  //theta
    cartes[Y_AXIS] = a_sin + b_sin + SCARA_OFFSET_Y;  //theta+phi

    /*
      SERIAL_ECHOPAIR("SCARA FK Angle a=", a);
      SERIAL_ECHOPAIR(" b=", b);
      SERIAL_ECHOPAIR(" a_sin=", a_sin);
      SERIAL_ECHOPAIR(" a_cos=", a_cos);
      SERIAL_ECHOPAIR(" b_sin=", b_sin);
      SERIAL_ECHOLNPAIR(" b_cos=", b_cos);
      SERIAL_ECHOPAIR(" cartes[X_AXIS]=", cartes[X_AXIS]);
      SERIAL_ECHOLNPAIR(" cartes[Y_AXIS]=", cartes[Y_AXIS]);
    //*/
  }

  /**
   * Morgan SCARA Inverse Kinematics. Results in delta[].
   *
   * See http://forums.reprap.org/read.php?185,283327
   *
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void inverse_kinematics(const float logical[XYZ]) {

    static float C2, S2, SK1, SK2, THETA, PSI;

    float sx = RAW_X_POSITION(logical[X_AXIS]) - SCARA_OFFSET_X,  // Translate SCARA to standard X Y
          sy = RAW_Y_POSITION(logical[Y_AXIS]) - SCARA_OFFSET_Y;  // With scaling factor.

    if (L1 == L2)
      C2 = HYPOT2(sx, sy) / L1_2_2 - 1;
    else
      C2 = (HYPOT2(sx, sy) - (L1_2 + L2_2)) / (2.0 * L1 * L2);

    S2 = sqrt(sq(C2) - 1);

    // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
    SK1 = L1 + L2 * C2;

    // Rotated Arm2 gives the distance from Arm1 to Arm2
    SK2 = L2 * S2;

    // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
    THETA = atan2(SK1, SK2) - atan2(sx, sy);

    // Angle of Arm2
    PSI = atan2(S2, C2);

    delta[A_AXIS] = DEGREES(THETA);        // theta is support arm angle
    delta[B_AXIS] = DEGREES(THETA + PSI);  // equal to sub arm angle (inverted motor)
    delta[C_AXIS] = logical[Z_AXIS];

    /*
      DEBUG_POS("SCARA IK", logical);
      DEBUG_POS("SCARA IK", delta);
      SERIAL_ECHOPAIR("  SCARA (x,y) ", sx);
      SERIAL_ECHOPAIR(",", sy);
      SERIAL_ECHOPAIR(" C2=", C2);
      SERIAL_ECHOPAIR(" S2=", S2);
      SERIAL_ECHOPAIR(" Theta=", THETA);
      SERIAL_ECHOLNPAIR(" Phi=", PHI);
    //*/
  }

#endif // MORGAN_SCARA

#if ENABLED(TEMP_STAT_LEDS)

  static bool red_led = false;
  static millis_t next_status_led_update_ms = 0;

  void handle_status_leds(void) {
    if (ELAPSED(millis(), next_status_led_update_ms)) {
      next_status_led_update_ms += 500; // Update every 0.5s
      float max_temp = 0.0;
      #if HAS_TEMP_BED
        max_temp = MAX3(max_temp, thermalManager.degTargetBed(), thermalManager.degBed());
      #endif
      HOTEND_LOOP() {
        max_temp = MAX3(max_temp, thermalManager.degHotend(e), thermalManager.degTargetHotend(e));
      }
      bool new_led = (max_temp > 55.0) ? true : (max_temp < 54.0) ? false : red_led;
      if (new_led != red_led) {
        red_led = new_led;
        #if PIN_EXISTS(STAT_LED_RED)
          WRITE(STAT_LED_RED_PIN, new_led ? HIGH : LOW);
          #if PIN_EXISTS(STAT_LED_BLUE)
            WRITE(STAT_LED_BLUE_PIN, new_led ? LOW : HIGH);
          #endif
        #else
          WRITE(STAT_LED_BLUE_PIN, new_led ? HIGH : LOW);
        #endif
      }
    }
  }

#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)

  void handle_filament_runout() {
    if (!filament_ran_out) {
      filament_ran_out = true;
      enqueue_and_echo_commands_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
      stepper.synchronize();
    }
  }

#endif // FILAMENT_RUNOUT_SENSOR

#if ENABLED(FAST_PWM_FAN)

  void setPwmFrequency(uint8_t pin, int val) {
    val &= 0x07;
    switch (digitalPinToTimer(pin)) {
      #if defined(TCCR0A)
        case TIMER0A:
        case TIMER0B:
          // TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
          // TCCR0B |= val;
          break;
      #endif
      #if defined(TCCR1A)
        case TIMER1A:
        case TIMER1B:
          // TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
          // TCCR1B |= val;
          break;
      #endif
      #if defined(TCCR2)
        case TIMER2:
        case TIMER2:
          TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
          TCCR2 |= val;
          break;
      #endif
      #if defined(TCCR2A)
        case TIMER2A:
        case TIMER2B:
          TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
          TCCR2B |= val;
          break;
      #endif
      #if defined(TCCR3A)
        case TIMER3A:
        case TIMER3B:
        case TIMER3C:
          TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
          TCCR3B |= val;
          break;
      #endif
      #if defined(TCCR4A)
        case TIMER4A:
        case TIMER4B:
        case TIMER4C:
          TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
          TCCR4B |= val;
          break;
      #endif
      #if defined(TCCR5A)
        case TIMER5A:
        case TIMER5B:
        case TIMER5C:
          TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
          TCCR5B |= val;
          break;
      #endif
    }
  }

#endif // FAST_PWM_FAN

//&begin[Extruder]
float calculate_volumetric_multiplier(float diameter) {
  if (!volumetric_enabled || diameter == 0) return 1.0;
  return 1.0 / (M_PI * diameter * 0.5 * diameter * 0.5);
}

void calculate_volumetric_multipliers() {
  for (uint8_t i = 0; i < COUNT(filament_size); i++)
    volumetric_multiplier[i] = calculate_volumetric_multiplier(filament_size[i]);
}
//&end[Extruder]

//&begin[Moter_Type_Stepper]
void enable_all_steppers() {
  enable_x();
  enable_y();
  enable_z();
  enable_e0();
  enable_e1();
  enable_e2();
  enable_e3();
}

void disable_all_steppers() {
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
  disable_e3();
}
//&end[Moter_Type_Stepper]

/**
 * Manage several activities:
 *  - Check for Filament Runout
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if pin CHDK needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 */
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {

  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    if ((IS_SD_PRINTING || print_job_timer.isRunning()) && (READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_INVERTING))
      handle_filament_runout();
  #endif

  if (commands_in_queue < BUFSIZE) get_available_commands();

  millis_t ms = millis();

  if (max_inactive_time && ELAPSED(ms, previous_cmd_ms + max_inactive_time)) kill(PSTR(MSG_KILLED));

  if (stepper_inactive_time && ELAPSED(ms, previous_cmd_ms + stepper_inactive_time)
      && !ignore_stepper_queue && !planner.blocks_queued()) {
    #if ENABLED(DISABLE_INACTIVE_X)
      disable_x();
    #endif
    #if ENABLED(DISABLE_INACTIVE_Y)
      disable_y();
    #endif
    #if ENABLED(DISABLE_INACTIVE_Z)
      disable_z();
    #endif
    #if ENABLED(DISABLE_INACTIVE_E)
      disable_e0();
      disable_e1();
      disable_e2();
      disable_e3();
    #endif
  }

  #ifdef CHDK // Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && PENDING(ms, chdkHigh + CHDK_DELAY)) {
      chdkActive = false;
      WRITE(CHDK, LOW);
    }
  #endif

  #if HAS_KILL

    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    static int killCount = 0;   // make the inactivity button a bit less responsive
    const int KILL_DELAY = 750;
    if (!READ(KILL_PIN))
      killCount++;
    else if (killCount > 0)
      killCount--;

    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if (killCount >= KILL_DELAY) kill(PSTR(MSG_KILLED));
  #endif

  #if HAS_HOME
    // Check to see if we have to home, use poor man's debouncer
    // ---------------------------------------------------------
    static int homeDebounceCount = 0;   // poor man's debouncing count
    const int HOME_DEBOUNCE_DELAY = 2500;
    if (!IS_SD_PRINTING && !READ(HOME_PIN)) {
      if (!homeDebounceCount) {
        enqueue_and_echo_commands_P(PSTR("G28"));
        LCD_MESSAGEPGM(MSG_AUTO_HOME);
      }
      if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
        homeDebounceCount++;
      else
        homeDebounceCount = 0;
    }
  #endif

  #if HAS_CONTROLLERFAN
    controllerFan(); // Check if fan should be turned on to cool stepper drivers down
  #endif

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    if (ELAPSED(ms, previous_cmd_ms + (EXTRUDER_RUNOUT_SECONDS) * 1000UL)
      && thermalManager.degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP) {
      bool oldstatus;
      #if ENABLED(SWITCHING_EXTRUDER)
        oldstatus = E0_ENABLE_READ;
        enable_e0();
      #else // !SWITCHING_EXTRUDER
        switch (active_extruder) {
          case 0:
            oldstatus = E0_ENABLE_READ;
            enable_e0();
            break;
          #if E_STEPPERS > 1
            case 1:
              oldstatus = E1_ENABLE_READ;
              enable_e1();
              break;
            #if E_STEPPERS > 2
              case 2:
                oldstatus = E2_ENABLE_READ;
                enable_e2();
                break;
              #if E_STEPPERS > 3
                case 3:
                  oldstatus = E3_ENABLE_READ;
                  enable_e3();
                  break;
              #endif
            #endif
          #endif
        }
      #endif // !SWITCHING_EXTRUDER

      previous_cmd_ms = ms; // refresh_cmd_timeout()

      #if IS_KINEMATIC
        inverse_kinematics(current_position);
        ADJUST_DELTA(current_position);
        planner.buffer_line(
          delta[A_AXIS], delta[B_AXIS], delta[C_AXIS],
          current_position[E_AXIS] + EXTRUDER_RUNOUT_EXTRUDE,
          MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED), active_extruder
        );
      #else
        planner.buffer_line(
          current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
          current_position[E_AXIS] + EXTRUDER_RUNOUT_EXTRUDE,
          MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED), active_extruder
        );
      #endif
      stepper.synchronize();
      planner.set_e_position_mm(current_position[E_AXIS]);
      #if ENABLED(SWITCHING_EXTRUDER)
        E0_ENABLE_WRITE(oldstatus);
      #else
        switch (active_extruder) {
          case 0:
            E0_ENABLE_WRITE(oldstatus);
            break;
          #if E_STEPPERS > 1
            case 1:
              E1_ENABLE_WRITE(oldstatus);
              break;
            #if E_STEPPERS > 2
              case 2:
                E2_ENABLE_WRITE(oldstatus);
                break;
              #if E_STEPPERS > 3
                case 3:
                  E3_ENABLE_WRITE(oldstatus);
                  break;
              #endif
            #endif
          #endif
        }
      #endif // !SWITCHING_EXTRUDER
    }
  #endif // EXTRUDER_RUNOUT_PREVENT

  #if ENABLED(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (delayed_move_time && ELAPSED(ms, delayed_move_time + 1000UL) && IsRunning()) {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      set_destination_to_current();
      prepare_move_to_destination();
    }
  #endif

  #if ENABLED(TEMP_STAT_LEDS)
    handle_status_leds();
  #endif

  planner.check_axes_activity();
}

/**
 * Standard idle routine keeps the machine alive
 */
void idle(
//&begin[FILAMENT_CHANGE_FEATURE]
  #if ENABLED(FILAMENT_CHANGE_FEATURE)
    bool no_stepper_sleep/*=false*/
  #endif
//&end[FILAMENT_CHANGE_FEATURE]
  ) {
  lcd_update();

  host_keepalive();

  #if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
    auto_report_temperatures();
  #endif

  manage_inactivity(
  //&begin[FILAMENT_CHANGE_FEATURE]
    #if ENABLED(FILAMENT_CHANGE_FEATURE)
      no_stepper_sleep
    #endif
	//&end[FILAMENT_CHANGE_FEATURE]
  );

  thermalManager.manage_heater();

  #if ENABLED(PRINTCOUNTER)
    print_job_timer.tick();
  #endif

  #if HAS_BUZZER && DISABLED(LCD_USE_I2C_BUZZER)
    buzzer.tick();
  #endif
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void kill(const char* lcd_msg) {
  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);

  #if ENABLED(ULTRA_LCD)
    kill_screen(lcd_msg);
  #else
    UNUSED(lcd_msg);
  #endif

  delay(500); // Wait a short time

  cli(); // Stop interrupts
  thermalManager.disable_all_heaters();
  disable_all_steppers();

  #if HAS_POWER_SWITCH
    SET_INPUT(PS_ON_PIN);
  #endif

  suicide();
  while (1) {
    #if ENABLED(USE_WATCHDOG)
      watchdog_reset();
    #endif
  } // Wait for reset
}

/**
 * Turn off heaters and stop the print in progress
 * After a stop the machine may be resumed with M999
 */
void stop() {
  thermalManager.disable_all_heaters();
  if (IsRunning()) {
    Running = false;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

/**
 * Marlin entry-point: Set up before the program loop
 *  - Set up the kill pin, filament runout, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    ?temperature
 *    ?planner
 *    ?watchdog
 *    ?stepper
 *    ?photo pin
 *    ?servos
 *    ?LCD controller
 *    ?Digipot I2C
 *    ?Z probe sled
 *    ?status LEDs
 */
void setup() {

  #ifdef DISABLE_JTAG
    // Disable JTAG on AT90USB chips to free up pins for IO
    MCUCR = 0x80;
    MCUCR = 0x80;
  #endif

  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    setup_filrunoutpin();
  #endif


  setup_killpin(); //&line[Board]


  setup_powerhold();//&line[Power_Supply]

  #if HAS_STEPPER_RESET
    disableStepperDrivers();
  #endif

//&begin[IO_Handling]
  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if (mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if (mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if (mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if (mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if (mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR = 0;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_CHAR(' ');
  SERIAL_ECHOLNPGM(SHORT_BUILD_VERSION);
  SERIAL_EOL;
//&end[IO_Handling]

  #if defined(STRING_DISTRIBUTION_DATE) && defined(STRING_CONFIG_H_AUTHOR)
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
    SERIAL_ECHOPGM(STRING_DISTRIBUTION_DATE);
    SERIAL_ECHOLNPGM(MSG_AUTHOR STRING_CONFIG_H_AUTHOR);
    SERIAL_ECHOLNPGM("Compiled: " __DATE__);
  #endif

//&begin[IO_Handling]
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR(MSG_FREE_MEMORY, freeMemory());
  SERIAL_ECHOLNPAIR(MSG_PLANNER_BUFFER_BYTES, (int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
//&end[IO_Handling]

  // Send "ok" after commands by default
//&line[Command_Handling]
  for (int8_t i = 0; i < BUFSIZE; i++) send_ok[i] = true;

  // Load data from EEPROM if available (or use defaults)
  // This also updates variables in the planner, elsewhere
  Config_RetrieveSettings();


  // Initialize current position based on home_offset
  memcpy(current_position, home_offset, sizeof(home_offset)); //&line[Move_To_Destination]


  // Vital to init stepper/planner equivalent for current_position
  SYNC_PLAN_POSITION_KINEMATIC(); //&line[Moter_Type_Stepper]


  thermalManager.init();  //&line[Temperature]   // Initialize temperature loop

  #if ENABLED(USE_WATCHDOG)
    watchdog_init();
  #endif


  stepper.init();  //&line[Moter_Type_Stepper]  // Initialize stepper, this enables interrupts!

  servo_init(); //&line[Moter_Type_Servo]

  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
//&begin[CASE_LIGHT]
  #if HAS_CASE_LIGHT
    update_case_light();
  #endif
//&end[CASE_LIGHT]
  #if HAS_BED_PROBE
    endstops.enable_z_probe(false);
  #endif

  #if HAS_CONTROLLERFAN
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif

  #if HAS_STEPPER_RESET
    enableStepperDrivers();
  #endif

  #if ENABLED(DIGIPOT_I2C)
    digipot_i2c_init();
  #endif

  #if ENABLED(DAC_STEPPER_CURRENT)
    dac_init();
  #endif

  #if ENABLED(Z_PROBE_SLED) && PIN_EXISTS(SLED)
    OUT_WRITE(SLED_PIN, LOW); // turn it off
  #endif // Z_PROBE_SLED


  setup_homepin(); //&line[Board]

  #if PIN_EXISTS(STAT_LED_RED)
    OUT_WRITE(STAT_LED_RED_PIN, LOW); // turn it off
  #endif

  #if PIN_EXISTS(STAT_LED_BLUE)
    OUT_WRITE(STAT_LED_BLUE_PIN, LOW); // turn it off
  #endif

  #if ENABLED(RGB_LED)
    pinMode(RGB_LED_R_PIN, OUTPUT);
    pinMode(RGB_LED_G_PIN, OUTPUT);
    pinMode(RGB_LED_B_PIN, OUTPUT);
  #endif

  lcd_init();
  #if ENABLED(SHOW_BOOTSCREEN)
    #if ENABLED(DOGLCD)
      safe_delay(BOOTSCREEN_TIMEOUT);
    #elif ENABLED(ULTRA_LCD)
      bootscreen();
      #if DISABLED(SDSUPPORT)
        lcd_init();
      #endif
    #endif
  #endif

//&begin[Extruder_Mixing]
  #if ENABLED(MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1
    // Initialize mixing to 100% color 1
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
      mixing_factor[i] = (i == 0) ? 1.0 : 0.0;
    for (uint8_t t = 0; t < MIXING_VIRTUAL_TOOLS; t++)
      for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
        mixing_virtual_tool_mix[t][i] = mixing_factor[i];
  #endif
//&end[Extruder_Mixing]

  #if ENABLED(EXPERIMENTAL_I2CBUS) && I2C_SLAVE_ADDRESS > 0
    i2c.onReceive(i2c_on_receive);
    i2c.onRequest(i2c_on_request);
  #endif

  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    setup_endstop_interrupts();
  #endif
}

/**
 * The main Marlin program loop
 *
 *  - Save or log commands to SD
 *  - Process available commands (if not saving)
 *  - Call heater manager
 *  - Call inactivity manager
 *  - Call endstop manager
 *  - Call LCD update
 */
void loop() {
//&begin[Command_Handling]  
if (commands_in_queue < BUFSIZE) get_available_commands();

  #if ENABLED(SDSUPPORT)
    card.checkautostart(false);
  #endif

  if (commands_in_queue) {

    #if ENABLED(SDSUPPORT)

      if (card.saving) {
        char* command = command_queue[cmd_queue_index_r];
        if (strstr_P(command, PSTR("M29"))) {
          // M29 closes the file
          card.closefile();
          SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
          ok_to_send();
        }
        else {
          // Write the string from the read buffer to SD
          card.write_command(command);
          if (card.logging)
            process_next_command(); // The card is saving because it's logging
          else
            ok_to_send();
        }
      }
      else
        process_next_command();

    #else

      process_next_command();

    #endif // SDSUPPORT

    // The queue may be reset by a command handler or by code invoked by idle() within a handler
    if (commands_in_queue) {
      --commands_in_queue;
      cmd_queue_index_r = (cmd_queue_index_r + 1) % BUFSIZE;
    }
  }
//&end[Command_Handling]

  endstops.report_state(); //&line[Endstop]
  idle();
}