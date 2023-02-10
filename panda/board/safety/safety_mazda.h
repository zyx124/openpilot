// CAN msgs we care about
#define MAZDA_LKAS          0x243
#define MAZDA_LKAS_HUD      0x440
#define MAZDA_CRZ_CTRL      0x21c
#define MAZDA_CRZ_BTNS      0x09d
#define MAZDA_STEER_TORQUE  0x240
#define MAZDA_ENGINE_DATA   0x202
#define MAZDA_PEDALS        0x165

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_AUX  1
#define MAZDA_CAM  2

#define MAZDA_MAX_STEER 2048U

// max delta torque allowed for real time checks
#define MAZDA_MAX_RT_DELTA 940
// 250ms between real time checks
#define MAZDA_RT_INTERVAL 250000
#define MAZDA_MAX_RATE_UP 10
#define MAZDA_MAX_RATE_DOWN 25
#define MAZDA_DRIVER_TORQUE_ALLOWANCE 15
#define MAZDA_DRIVER_TORQUE_FACTOR 1
#define MAZDA_MAX_TORQUE_ERROR 350

const CanMsg MAZDA_TX_MSGS[] = {{MAZDA_LKAS, 0, 8}, {MAZDA_CRZ_BTNS, 0, 8}, {MAZDA_LKAS_HUD, 0, 8}};

AddrCheckStruct mazda_addr_checks[] = {
  {.msg = {{MAZDA_CRZ_CTRL,     0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{MAZDA_CRZ_BTNS,     0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{MAZDA_STEER_TORQUE, 0, 8, .expected_timestep = 12000U}, { 0 }, { 0 }}},
  {.msg = {{MAZDA_ENGINE_DATA,  0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{MAZDA_PEDALS,       0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define MAZDA_ADDR_CHECKS_LEN (sizeof(mazda_addr_checks) / sizeof(mazda_addr_checks[0]))
addr_checks mazda_rx_checks = {mazda_addr_checks, MAZDA_ADDR_CHECKS_LEN};

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static int mazda_rx_hook(CANPacket_t *to_push) {
  bool valid = addr_safety_check(to_push, &mazda_rx_checks, NULL, NULL, NULL);
  if (valid && ((int)GET_BUS(to_push) == MAZDA_MAIN)) {
    int addr = GET_ADDR(to_push);

    if (addr == MAZDA_ENGINE_DATA) {
      // sample speed: scale by 0.01 to get kph
      int speed = (GET_BYTE(to_push, 2) << 8) | GET_BYTE(to_push, 3);
      vehicle_moving = speed > 10; // moving when speed > 0.1 kph
    }

    if (addr == MAZDA_STEER_TORQUE) {
      int torque_driver_new = GET_BYTE(to_push, 0) - 127U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (addr == MAZDA_CRZ_CTRL) {
      bool cruise_engaged = GET_BYTE(to_push, 0) & 0x8U;
      if (cruise_engaged) {
        if (!cruise_engaged_prev) {
          controls_allowed = 1;
        }
      } else {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    if (addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (GET_BYTE(to_push, 4) || (GET_BYTE(to_push, 5) & 0xF0U));
    }

    if (addr == MAZDA_PEDALS) {
      brake_pressed = (GET_BYTE(to_push, 0) & 0x10U);
    }

    generic_rx_checks((addr == MAZDA_LKAS));
  }
  return valid;
}

static int mazda_tx_hook(CANPacket_t *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  if (!msg_allowed(to_send, MAZDA_TX_MSGS, sizeof(MAZDA_TX_MSGS)/sizeof(MAZDA_TX_MSGS[0]))) {
    tx = 0;
  }

  // Check if msg is sent on the main BUS
  if (bus == MAZDA_MAIN) {

    // steer cmd checks
    if (addr == MAZDA_LKAS) {
      int desired_torque = (((GET_BYTE(to_send, 0) & 0x0FU) << 8) | GET_BYTE(to_send, 1)) - MAZDA_MAX_STEER;
      bool violation = 0;
      uint32_t ts = microsecond_timer_get();

      if (controls_allowed) {

        // *** global torque limit check ***
        violation |= max_limit_check(desired_torque, MAZDA_MAX_STEER, -MAZDA_MAX_STEER);

        // *** torque rate limit check ***
        violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
                                        MAZDA_MAX_STEER, MAZDA_MAX_RATE_UP, MAZDA_MAX_RATE_DOWN,
                                        MAZDA_DRIVER_TORQUE_ALLOWANCE, MAZDA_DRIVER_TORQUE_FACTOR);

        // used next time
        desired_torque_last = desired_torque;

        // *** torque real time rate limit check ***
        violation |= rt_rate_limit_check(desired_torque, rt_torque_last, MAZDA_MAX_RT_DELTA);

        // every RT_INTERVAL set the new limits
        uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
        if (ts_elapsed > ((uint32_t) MAZDA_RT_INTERVAL)) {
          rt_torque_last = desired_torque;
          ts_last = ts;
        }
      }

      // no torque if controls is not allowed
      if (!controls_allowed && (desired_torque != 0)) {
        violation = 1;
      }

      // reset to 0 if either controls is not allowed or there's a violation
      if (violation || !controls_allowed) {
        desired_torque_last = 0;
        rt_torque_last = 0;
        ts_last = ts;
      }

      if (violation) {
        tx = 0;
      }
    }

    // cruise buttons check
    if (addr == MAZDA_CRZ_BTNS) {
      // allow resume spamming while controls allowed, but
      // only allow cancel while contrls not allowed
      bool cancel_cmd = (GET_BYTE(to_send, 0) == 0x1U);
      if (!controls_allowed && !cancel_cmd) {
        tx = 0;
      }
    }
  }

  return tx;
}

static int mazda_fwd_hook(int bus, CANPacket_t *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  if (bus == MAZDA_MAIN) {
    bus_fwd = MAZDA_CAM;
  } else if (bus == MAZDA_CAM) {
    bool block = (addr == MAZDA_LKAS) || (addr == MAZDA_LKAS_HUD);
    if (!block) {
      bus_fwd = MAZDA_MAIN;
    }
  } else {
    // don't fwd
  }

  return bus_fwd;
}

static const addr_checks* mazda_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();
  return &mazda_rx_checks;
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = mazda_fwd_hook,
};

/*
  Mazda Gen 4 2019+
  Mazda 3 2019+
  CX-30,50,90
*/ 
#define MAZDA_2019_BRAKE          0x43F // main bus
#define MAZDA_2019_GAS            0x202 // camera bus DBC: ENGINE_DATA
#define MAZDA_2019_CRUISE         0x44A // main bus. DBC: CRUISE_STATE
#define MAZDA_2019_SPEED          0x217 // camera bus. DBC: SPEED 
#define MAZDA_2019_STEER_TORQUE   0x24B // aux bus. DBC: EPS_FEEDBACK
#define MAZDA_2019_LKAS           0x249 // aux bus. DBC: EPS_LKAS
#define MAZDA_2019_CRZ_BTNS       0x9d  // rx on main tx on camera. DBC: CRZ_BTNS
#define MAZDA_2019_ACC            0x220 // main bus. DBC: ACC

#define MAZDA_2019_MAX_STEER 8000U

// max delta torque allowed for real time checks
#define MAZDA_2019_MAX_RT_DELTA 844
// 250ms between real time checks
#define MAZDA_2019_RT_INTERVAL 250000
#define MAZDA_2019_MAX_RATE_UP 45
#define MAZDA_2019_MAX_RATE_DOWN 80
#define MAZDA_2019_DRIVER_TORQUE_ALLOWANCE 1400
#define MAZDA_2019_DRIVER_TORQUE_FACTOR 1
#define MAZDA_2019_MAX_TORQUE_ERROR 3500

const CanMsg MAZDA_2019_TX_MSGS[] = {{MAZDA_2019_LKAS, 1, 8}, {MAZDA_2019_ACC, 2, 8}};

AddrCheckStruct mazda_2019_addr_checks[] = {
  {.msg = {{MAZDA_2019_BRAKE,     0, 8, .expected_timestep = 50000U}, { 0 }, { 0 }}},
  {.msg = {{MAZDA_2019_GAS,       2, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{MAZDA_2019_CRUISE,    0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{MAZDA_2019_SPEED,     2, 8, .expected_timestep = 30000U}, { 0 }, { 0 }}},
  {.msg = {{MAZDA_2019_STEER_TORQUE,     1, 8, .expected_timestep = 50000U}, { 0 }, { 0 }}},
};

#define MAZDA_2019_ADDR_CHECKS_LEN (sizeof(mazda_2019_addr_checks) / sizeof(mazda_2019_addr_checks[0]))
addr_checks mazda_2019_rx_checks = {mazda_2019_addr_checks, MAZDA_2019_ADDR_CHECKS_LEN};

static int mazda_2019_rx_hook(CANPacket_t *to_push) {
  bool valid = addr_safety_check(to_push, &mazda_2019_rx_checks, NULL, NULL, NULL);
  static bool cruise_engaged;
  static int speed;
  if (valid) {
    int bus = GET_BUS(to_push);
    int addr = GET_ADDR(to_push);
    switch (bus) {
      case MAZDA_MAIN:
        switch (addr) {
          case MAZDA_2019_BRAKE:
            brake_pressed = (GET_BYTE(to_push, 5) & 0x4U);
            break; // end MAZDA_2019_BRAKE

          case MAZDA_2019_CRUISE:
            cruise_engaged = GET_BYTE(to_push, 0) & 0x20U;
            bool pre_enable = GET_BYTE(to_push, 0) & 0x40U;
            pcm_cruise_check((cruise_engaged || pre_enable));
            break; // end MAZDA_2019_CRUISE

          default: // default address main
            break;
        }
        generic_rx_checks((addr == MAZDA_2019_GAS)); 
        break; // end MAZDA_MAIN

      case MAZDA_CAM:
        switch (addr) {
          case MAZDA_2019_GAS:
            gas_pressed = (GET_BYTE(to_push, 4) || (GET_BYTE(to_push, 5) & 0xC0U));
            break; // end MAZDA_2019_GAS

          case MAZDA_2019_SPEED:
            // sample speed: scale by 0.01 to get kph
            speed = (GET_BYTE(to_push, 4) << 8) | (GET_BYTE(to_push, 5));
            vehicle_moving = (speed > 10); // moving when speed > 0.1 kph
            break; // end MAZDA_2019_SPEED
          
          default: // default address cam
            break;
        }
        break; // end MAZDA_CAM

      case MAZDA_AUX:
        switch (addr) {
          case MAZDA_2019_STEER_TORQUE:
            update_sample(&torque_driver, (int16_t)(GET_BYTE(to_push, 0) << 8 | GET_BYTE(to_push, 1)));
            break; // end TI2_STEER_TORQUE
          
          default: // default address aux
            break;
        }
        break; // end MAZDA_AUX 

      default: // default bus
        break;
    }
    
  }

  return valid;
}

static int mazda_2019_tx_hook(CANPacket_t *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  if (!msg_allowed(to_send, MAZDA_2019_TX_MSGS, sizeof(MAZDA_2019_TX_MSGS)/sizeof(MAZDA_2019_TX_MSGS[0]))) {
    tx = 0;
  }
  if (bus == MAZDA_AUX) {
    if (addr == MAZDA_2019_LKAS) {
      int desired_torque = (int16_t)((GET_BYTE(to_send, 0) << 8) | GET_BYTE(to_send, 1)); // signal is signed
      bool violation = 0;
      uint32_t ts = microsecond_timer_get();

      if (controls_allowed) {

        // *** global torque limit check ***
        violation |= max_limit_check(desired_torque, MAZDA_MAX_STEER, -MAZDA_MAX_STEER);

        // *** torque rate limit check ***
        violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
                                        MAZDA_MAX_STEER, MAZDA_MAX_RATE_UP, MAZDA_MAX_RATE_DOWN,
                                        MAZDA_DRIVER_TORQUE_ALLOWANCE, MAZDA_DRIVER_TORQUE_FACTOR);

        // used next time
        desired_torque_last = desired_torque;

        // *** torque real time rate limit check ***
        violation |= rt_rate_limit_check(desired_torque, rt_torque_last, MAZDA_MAX_RT_DELTA);

        // every RT_INTERVAL set the new limits
        uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
        if (ts_elapsed > ((uint32_t) MAZDA_RT_INTERVAL)) {
          rt_torque_last = desired_torque;
          ts_last = ts;
        }
      }

      // no torque if controls is not allowed
      if (!controls_allowed && (desired_torque != 0)) {
        violation = 1;
      }

      // reset to 0 if either controls is not allowed or there's a violation
      if (violation || !controls_allowed) {
        desired_torque_last = 0;
        rt_torque_last = 0;
        ts_last = ts;
      }

      if (violation) {
        tx = 0;
      }
    }
  }
  return tx;
}

static int mazda_2019_fwd_hook(int bus, CANPacket_t *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);
  bool block = false;
  
  if (bus == MAZDA_MAIN) {
    block = (addr == MAZDA_2019_ACC);
    if (!block) {
      bus_fwd = MAZDA_CAM;
    }

  } else if (bus == MAZDA_CAM) {
    if (!block) {
      bus_fwd = MAZDA_MAIN;
    }
  } else {
    // don't fwd
  }

  return bus_fwd;
}

static const addr_checks* mazda_2019_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();
  return &mazda_2019_rx_checks;
}

const safety_hooks mazda_2019_hooks = {
  .init = mazda_2019_init,
  .rx = mazda_2019_rx_hook,
  .tx = mazda_2019_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = mazda_2019_fwd_hook,
};