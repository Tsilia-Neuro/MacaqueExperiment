import os
import time
import random
import csv
import logging
import sys
from pathlib import Path
from datetime import datetime
from telemetrix import telemetrix


# ============================================================
# EXPERIMENTAL PARAMETERS
# ============================================================

DATA_FOLDER_NAME = "Experiment_Decision_Making_Pressure"

SESSION_DURATION_SEC = 2 * 3600   # 2 hours total
NUM_TRIALS = 5
TRIAL_DURATION_SEC = 10.0         # Mandatory 10-second window

# Short ITI values for testing
MIN_ITI = 5
MAX_ITI = 10

# ----------------------------
# Trial polling
# ----------------------------
TRIAL_POLL_INTERVAL_SEC = 0.005

# ----------------------------
# Reward timing configuration
# ----------------------------
USE_RANDOM_REWARD_DELAYS = False

SERVO_DELAY_FIXED_SEC = 1.0       # Servo starts 1.0 s after correct press
STEPPER_DELAY_FIXED_SEC = 1.2     # Stepper starts 1.2 s after correct press

SERVO_HOLD_FIXED_SEC = 4.0        # Servo stays at target position for 4.0 s

# Example ranges for future random delays
SERVO_DELAY_RANGE_SEC = (0.8, 1.5)
STEPPER_DELAY_RANGE_SEC = (1.0, 1.8)
SERVO_HOLD_RANGE_SEC = (3.5, 4.5)

# ----------------------------
# Hardware pins
# ----------------------------
PIN_SERVO = 3
PIN_BTN_M1 = 4
PIN_BTN_M2 = 5
PIN_LED_M1 = 6
PIN_LED_M2 = 7
PIN_STEPPER = [8, 9, 10, 11]  # IN1, IN2, IN3, IN4

PIN_BUZZER = 12
BUZZER_FREQUENCY_HZ = 1000

# Short stimulus beep duration
BUZZER_DURATION_MS = 1000

# Long beep after correct press
CORRECT_PRESS_BUZZER_DURATION_MS = 2000

# Silence gap when the correct press happens during the first beep
BUZZER_SWITCH_GAP_MS = 200

# ----------------------------
# Servo angles
# ----------------------------
ANGLE_CENTER = 120
ANGLE_M1 = 160
ANGLE_M2 = 90

# ----------------------------
# Stepper parameters
# 28BYJ-48 + ULN2003
# ----------------------------
STEPS_PER_REV = 2048
STEPS_90_DEG = 300
STEPPER_TEST_STEPS = 20
STEPPER_MAX_SPEED = 260
STEPPER_ACCELERATION = 140
STEPPER_TIMEOUT_SEC = 8.0

RUN_STEPPER_TEST_AT_STARTUP = True


# ============================================================
# LOGGING / ENVIRONMENT
# ============================================================

def setup_environment():
    """
    Creates the data folder on Desktop or Bureau and configures logging.
    """
    home = Path.home()
    desktop = home / "Desktop"

    if not desktop.exists():
        desktop = home / "Bureau"

    if not desktop.exists():
        desktop = home

    data_dir = desktop / DATA_FOLDER_NAME
    data_dir.mkdir(parents=True, exist_ok=True)

    log_file = data_dir / "experiment_system.log"

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        handlers=[
            logging.FileHandler(log_file, encoding="utf-8"),
            logging.StreamHandler(sys.stdout)
        ]
    )

    logging.info(f"Working folder configured: {data_dir}")
    return data_dir


# ============================================================
# OPERATOR INPUT HELPERS
# ============================================================

def ask_macaque():
    while True:
        answer = input("Which macaque is being tested? [M1/M2]: ").strip().upper()
        if answer in {"M1", "M2"}:
            return answer
        print("Invalid value. Please enter exactly 'M1' or 'M2'.")


def ask_wall_present():
    valid_true = {"YES", "Y", "TRUE", "T", "1", "YES"}
    valid_false = {"NO", "N", "FALSE", "F", "0", "NO"}

    while True:
        answer = input("Is the wall present? [Yes/No]: ").strip().upper()
        if answer in valid_true:
            return True
        if answer in valid_false:
            return False
        print("Invalid value. Please answer Yes/No, True/False, or equivalent.")


def ask_session_number():
    while True:
        answer = input("What is the session number today? [1-10]: ").strip()
        if answer.isdigit():
            value = int(answer)
            if 1 <= value <= 10:
                return value
        print("Invalid value. Please enter an integer between 1 and 10.")


# ============================================================
# MAIN CLASS
# ============================================================

class MacaqueExperiment:
    def __init__(self, data_dir, port=None):
        self.data_dir = Path(data_dir)

        logging.info("Initializing experiment and connecting to board...")

        try:
            self.board = telemetrix.Telemetrix(
                com_port=port,
                arduino_wait=8
            )
        except Exception as e:
            logging.exception(f"Unable to connect to Arduino/Telemetrix: {e}")
            raise

        # Trial/session state
        self.in_trial = False
        self.in_iti = False
        self.active_side = None
        self.correct_button_pin = None
        self.correct_led_pin = None

        self.trial_start_perf = None
        self.trial_start_timestamp = None
        self.stimulus_on_timestamp = None
        self.button_press_timestamp = None
        self.reward_delivery_timestamp = None

        self.button_pressed = False
        self.wrong_button_pressed = False
        self.omission = False
        self.premature_iti_presses = 0
        self.pressed_pin = None
        self.reaction_time_ms = int(TRIAL_DURATION_SEC * 1000)

        # INPUT_PULLUP => released = 1, pressed = 0
        self.btn_state = {
            PIN_BTN_M1: 1,
            PIN_BTN_M2: 1
        }

        # Stepper completion tracking
        self.stepper_done = False

        # CSV path
        self.current_csv_path = None

        # Telemetrix motor ID
        self.motor_id = None

        # Buzzer status
        self.buzzer_ready = False

        # Stimulus buzzer state
        self.stimulus_buzzer_active = False
        self.stimulus_buzzer_end_time_perf = None

        # Correct press long buzzer state
        self.correct_press_buzzer_active = False
        self.correct_press_buzzer_end_time_perf = None
        self.correct_press_buzzer_pending = False
        self.correct_press_buzzer_start_time_perf = None

        # Delayed reward scheduling state
        self.reward_schedule_active = False
        self.reward_target_side = None
        self.press_perf_time = None

        self.current_servo_delay_sec = None
        self.current_stepper_delay_sec = None
        self.current_servo_hold_sec = None

        self.servo_activation_time_perf = None
        self.stepper_activation_time_perf = None
        self.servo_return_time_perf = None

        self.servo_activated = False
        self.stepper_activated = False
        self.servo_returned = False

        self.setup_hardware()

    # --------------------------------------------------------
    # Timing configuration helpers
    # --------------------------------------------------------
    def get_trial_reward_timing(self):
        """
        Returns servo_delay_sec, stepper_delay_sec, servo_hold_sec.
        """
        if USE_RANDOM_REWARD_DELAYS:
            servo_delay = random.uniform(*SERVO_DELAY_RANGE_SEC)
            stepper_delay = random.uniform(*STEPPER_DELAY_RANGE_SEC)
            servo_hold = random.uniform(*SERVO_HOLD_RANGE_SEC)
        else:
            servo_delay = SERVO_DELAY_FIXED_SEC
            stepper_delay = STEPPER_DELAY_FIXED_SEC
            servo_hold = SERVO_HOLD_FIXED_SEC

        return servo_delay, stepper_delay, servo_hold

    # --------------------------------------------------------
    # Buzzer helpers
    # --------------------------------------------------------
    def start_stimulus_buzzer(self):
        """
        Starts the short stimulus beep in a non-blocking way.
        """
        if not self.buzzer_ready:
            return

        try:
            self.board.digital_write(PIN_BUZZER, 1)
            self.stimulus_buzzer_active = True
            self.stimulus_buzzer_end_time_perf = (
                time.perf_counter() + BUZZER_DURATION_MS / 1000.0
            )

            logging.info(f"Stimulus buzzer started for {BUZZER_DURATION_MS} ms")

        except Exception as e:
            logging.warning(f"Error starting stimulus buzzer: {e}")

    def update_stimulus_buzzer(self):
        """
        Stops the short stimulus buzzer when its duration is finished.
        """
        if not self.stimulus_buzzer_active:
            return

        if time.perf_counter() >= self.stimulus_buzzer_end_time_perf:
            self.stop_stimulus_buzzer()

    def stop_stimulus_buzzer(self):
        """
        Stops the stimulus buzzer immediately.
        """
        if not self.stimulus_buzzer_active:
            return

        try:
            self.board.digital_write(PIN_BUZZER, 0)
        except Exception as e:
            logging.warning(f"Error stopping stimulus buzzer: {e}")

        self.stimulus_buzzer_active = False
        self.stimulus_buzzer_end_time_perf = None

        logging.info("Stimulus buzzer stopped.")

    def start_correct_press_buzzer(self):
        """
        Starts a longer beep when the correct macaque presses the correct button
        during the active stimulus window.

        If the first stimulus beep is still active, it is stopped first,
        then the long beep starts after a short silence.
        """
        if not self.buzzer_ready:
            return

        if self.stimulus_buzzer_active:
            self.stop_stimulus_buzzer()

            self.correct_press_buzzer_pending = True
            self.correct_press_buzzer_start_time_perf = (
                time.perf_counter() + BUZZER_SWITCH_GAP_MS / 1000.0
            )

            logging.info(
                f"Correct press long buzzer pending. "
                f"Gap before start = {BUZZER_SWITCH_GAP_MS} ms"
            )
            return

        self.start_correct_press_buzzer_now()

    def start_correct_press_buzzer_now(self):
        """
        Starts the correct press long buzzer immediately.
        """
        if not self.buzzer_ready:
            return

        try:
            self.board.digital_write(PIN_BUZZER, 1)

            self.correct_press_buzzer_active = True
            self.correct_press_buzzer_pending = False
            self.correct_press_buzzer_start_time_perf = None

            self.correct_press_buzzer_end_time_perf = (
                time.perf_counter() + CORRECT_PRESS_BUZZER_DURATION_MS / 1000.0
            )

            logging.info(
                f"Correct press long buzzer started for "
                f"{CORRECT_PRESS_BUZZER_DURATION_MS} ms"
            )

        except Exception as e:
            logging.warning(f"Error starting correct press buzzer: {e}")

    def update_correct_press_buzzer(self):
        """
        Starts the pending correct press buzzer after the silence gap,
        then stops it when its duration is finished.
        """
        if self.correct_press_buzzer_pending:
            if time.perf_counter() >= self.correct_press_buzzer_start_time_perf:
                self.start_correct_press_buzzer_now()
            return

        if not self.correct_press_buzzer_active:
            return

        if time.perf_counter() >= self.correct_press_buzzer_end_time_perf:
            try:
                self.board.digital_write(PIN_BUZZER, 0)
            except Exception as e:
                logging.warning(f"Error stopping correct press buzzer: {e}")

            self.correct_press_buzzer_active = False
            self.correct_press_buzzer_end_time_perf = None

            logging.info("Correct press long buzzer stopped.")

    def trigger_stimulus_cue(self, side):
        """
        Activates the visual stimulus and starts the short stimulus buzzer.
        """
        self.light_only_led_for_side(side)
        self.start_stimulus_buzzer()

    # --------------------------------------------------------
    # Timestamp helper
    # --------------------------------------------------------
    def current_timestamp(self):
        return datetime.now().isoformat(
            sep=" ",
            timespec="milliseconds"
        )

    # --------------------------------------------------------
    # Mapping helpers
    # --------------------------------------------------------
    def button_pin_for_side(self, side):
        return PIN_BTN_M1 if side == "M1" else PIN_BTN_M2

    def led_pin_for_side(self, side):
        return PIN_LED_M1 if side == "M1" else PIN_LED_M2

    def servo_angle_for_side(self, side):
        return ANGLE_M1 if side == "M1" else ANGLE_M2

    def other_side(self, side):
        return "M2" if side == "M1" else "M1"

    # --------------------------------------------------------
    # Reward rule helper
    # --------------------------------------------------------
    def get_reward_recipient(self):
        """
        Reward rule:
        - If the tested macaque presses the correct button,
          reward goes to the tested macaque.
        - If the tested macaque does not press the correct button,
          reward goes to the other macaque.
        """
        if self.button_pressed:
            return self.active_side
        return self.other_side(self.active_side)

    # --------------------------------------------------------
    # Hardware setup
    # --------------------------------------------------------
    def setup_hardware(self):
        logging.info("Configuring pins...")

        # LEDs
        self.board.set_pin_mode_digital_output(PIN_LED_M1)
        time.sleep(0.05)

        self.board.set_pin_mode_digital_output(PIN_LED_M2)
        time.sleep(0.05)

        self.board.digital_write(PIN_LED_M1, 0)
        self.board.digital_write(PIN_LED_M2, 0)

        # Servo
        self.board.set_pin_mode_servo(PIN_SERVO, 100, 3000)
        time.sleep(0.2)

        self.board.servo_write(PIN_SERVO, ANGLE_CENTER)
        time.sleep(0.5)

        # Stepper
        self.motor_id = self.board.set_pin_mode_stepper(
            interface=4,
            pin1=PIN_STEPPER[0],
            pin2=PIN_STEPPER[1],
            pin3=PIN_STEPPER[2],
            pin4=PIN_STEPPER[3]
        )
        time.sleep(0.2)

        self.board.stepper_set_max_speed(self.motor_id, STEPPER_MAX_SPEED)
        self.board.stepper_set_acceleration(self.motor_id, STEPPER_ACCELERATION)
        self.board.stepper_set_current_position(self.motor_id, 0)
        time.sleep(0.2)

        # Buttons with pull-up callbacks
        self.board.set_pin_mode_digital_input_pullup(
            PIN_BTN_M1,
            callback=self.button_callback
        )
        time.sleep(0.05)

        self.board.set_pin_mode_digital_input_pullup(
            PIN_BTN_M2,
            callback=self.button_callback
        )
        time.sleep(0.2)

        # Active buzzer
        try:
            self.board.set_pin_mode_digital_output(PIN_BUZZER)
            self.board.digital_write(PIN_BUZZER, 0)
            self.buzzer_ready = True
            logging.info(f"Active buzzer ready on pin {PIN_BUZZER}")
        except Exception as e:
            self.buzzer_ready = False
            logging.warning(f"Unable to initialize the active buzzer: {e}")

        logging.info(f"Hardware ready. Stepper motor_id = {self.motor_id}")

        if RUN_STEPPER_TEST_AT_STARTUP:
            self.test_stepper()

    # --------------------------------------------------------
    # LED helpers
    # --------------------------------------------------------
    def light_only_led_for_side(self, side):
        self.board.digital_write(PIN_LED_M1, 1 if side == "M1" else 0)
        self.board.digital_write(PIN_LED_M2, 1 if side == "M2" else 0)

    def leds_off(self):
        self.board.digital_write(PIN_LED_M1, 0)
        self.board.digital_write(PIN_LED_M2, 0)

    # --------------------------------------------------------
    # Sleep helper
    # --------------------------------------------------------
    def safe_sleep(self, duration_sec):
        start = time.perf_counter()

        while (time.perf_counter() - start) < duration_sec:
            time.sleep(0.01)

    # --------------------------------------------------------
    # Scheduled reward helpers
    # --------------------------------------------------------
    def reset_scheduled_reward_state(self):
        self.reward_schedule_active = False
        self.reward_target_side = None
        self.press_perf_time = None

        self.current_servo_delay_sec = None
        self.current_stepper_delay_sec = None
        self.current_servo_hold_sec = None

        self.servo_activation_time_perf = None
        self.stepper_activation_time_perf = None
        self.servo_return_time_perf = None

        self.servo_activated = False
        self.stepper_activated = False
        self.servo_returned = False

    def schedule_delayed_reward(
        self,
        target_side,
        press_perf_time,
        servo_delay_sec,
        stepper_delay_sec,
        servo_hold_sec
    ):
        """
        Schedules the reward components after a correct button press.
        """
        self.reward_schedule_active = True
        self.reward_target_side = target_side
        self.press_perf_time = press_perf_time

        self.current_servo_delay_sec = servo_delay_sec
        self.current_stepper_delay_sec = stepper_delay_sec
        self.current_servo_hold_sec = servo_hold_sec

        self.servo_activation_time_perf = press_perf_time + servo_delay_sec
        self.stepper_activation_time_perf = press_perf_time + stepper_delay_sec
        self.servo_return_time_perf = (
            self.servo_activation_time_perf + servo_hold_sec
        )

        self.servo_activated = False
        self.stepper_activated = False
        self.servo_returned = False

        logging.info(
            f"Reward scheduled for {target_side} | "
            f"servo_delay={servo_delay_sec:.3f}s | "
            f"stepper_delay={stepper_delay_sec:.3f}s | "
            f"servo_hold={servo_hold_sec:.3f}s"
        )

    # --------------------------------------------------------
    # Stepper helpers
    # --------------------------------------------------------
    def stepper_power_off(self):
        """
        Turns off the stepper coils between rewards.
        This prevents the motor from staying in holding mode.
        """
        try:
            for pin in PIN_STEPPER:
                self.board.digital_write(pin, 0)

            logging.info("Stepper coils turned off between rewards.")

        except Exception as e:
            logging.warning(f"Unable to turn off stepper coils: {e}")

    def run_stepper_once(self):
        """
        Runs the stepper once and waits for completion.
        Then turns off the stepper coils so the motor does not stay holding.
        """
        self.stepper_done = False

        self.board.stepper_move(self.motor_id, STEPS_90_DEG)
        self.board.stepper_run(
            self.motor_id,
            completion_callback=self.stepper_completion_callback
        )

        start_wait = time.perf_counter()

        while (
            not self.stepper_done
            and (time.perf_counter() - start_wait) < STEPPER_TIMEOUT_SEC
        ):
            time.sleep(0.02)

        if not self.stepper_done:
            logging.warning("Stepper timeout reached before completion callback.")

        # Turn off stepper coils after each reward movement
        self.stepper_power_off()

    def update_scheduled_reward(self):
        """
        Non-blocking scheduler for delayed servo/stepper actions after a correct press.
        """
        if not self.reward_schedule_active:
            return

        now = time.perf_counter()

        # Servo activation
        if not self.servo_activated and now >= self.servo_activation_time_perf:
            target_angle = self.servo_angle_for_side(self.reward_target_side)
            self.board.servo_write(PIN_SERVO, target_angle)
            self.servo_activated = True

            logging.info(
                f"Servo moved to {self.reward_target_side} | "
                f"target_angle={target_angle} | "
                f"delay_from_press={self.current_servo_delay_sec:.3f}s"
            )

        # Stepper activation
        if not self.stepper_activated and now >= self.stepper_activation_time_perf:
            self.reward_delivery_timestamp = self.current_timestamp()

            logging.info(
                f"Stepper reward triggered for {self.reward_target_side} | "
                f"Reward_Delivery_Timestamp = {self.reward_delivery_timestamp} | "
                f"delay_from_press={self.current_stepper_delay_sec:.3f}s"
            )

            self.run_stepper_once()
            self.stepper_activated = True

        # Servo return
        if (
            self.servo_activated
            and not self.servo_returned
            and now >= self.servo_return_time_perf
        ):
            self.board.servo_write(PIN_SERVO, ANGLE_CENTER)
            self.servo_returned = True

            logging.info(
                f"Servo returned to center | "
                f"hold_duration={self.current_servo_hold_sec:.3f}s"
            )

        # Schedule complete
        if self.servo_activated and self.stepper_activated and self.servo_returned:
            self.reward_schedule_active = False
            logging.info("Scheduled reward sequence completed.")

    # --------------------------------------------------------
    # Button event processing
    # --------------------------------------------------------
    def process_button_press(self, pin_number):
        """
        Processes a confirmed press transition on a button.
        This function is called only when the input changes
        from released=1 to pressed=0.
        """
        button_event_timestamp = self.current_timestamp()

        logging.info(
            f"Button press transition detected on pin {pin_number} | "
            f"in_iti={self.in_iti} | "
            f"in_trial={self.in_trial} | "
            f"active_side={self.active_side} | "
            f"correct_button_pin={self.correct_button_pin} | "
            f"Timestamp={button_event_timestamp}"
        )

        # Premature ITI press
        if self.in_iti and not self.in_trial:
            self.premature_iti_presses += 1
            logging.info(
                f"Premature ITI button press detected on pin {pin_number} | "
                f"Premature_ITI_Presses = {self.premature_iti_presses}"
            )
            return

        # Outside trial => ignore for experimental scoring
        if not self.in_trial:
            return

        # Store timestamp of first button press during the trial
        if self.button_press_timestamp is None:
            self.button_press_timestamp = button_event_timestamp

        # Correct button
        if pin_number == self.correct_button_pin and not self.button_pressed:
            self.reaction_time_ms = int(
                (time.perf_counter() - self.trial_start_perf) * 1000
            )
            self.button_pressed = True
            self.pressed_pin = pin_number

            # If the first stimulus beep is still active, it stops it,
            # waits BUZZER_SWITCH_GAP_MS, then starts the long beep.
            self.start_correct_press_buzzer()

            logging.info(
                f"[{self.active_side}] Correct button press detected "
                f"on pin {pin_number} at {self.reaction_time_ms} ms | "
                f"Button_Press_Timestamp = {button_event_timestamp}"
            )

            # Immediate visual feedback
            self.board.digital_write(self.correct_led_pin, 0)

            # Schedule delayed reward components
            press_perf_time = time.perf_counter()
            servo_delay_sec, stepper_delay_sec, servo_hold_sec = (
                self.get_trial_reward_timing()
            )

            self.schedule_delayed_reward(
                target_side=self.active_side,
                press_perf_time=press_perf_time,
                servo_delay_sec=servo_delay_sec,
                stepper_delay_sec=stepper_delay_sec,
                servo_hold_sec=servo_hold_sec
            )

        # Wrong button
        elif pin_number != self.correct_button_pin:
            self.wrong_button_pressed = True

            logging.info(
                f"[{self.active_side}] Wrong button pressed on pin {pin_number} | "
                f"Expected correct pin = {self.correct_button_pin} | "
                f"Button_Press_Timestamp = {button_event_timestamp} "
                f"(ignored for reward)."
            )

    # --------------------------------------------------------
    # Button callback
    # --------------------------------------------------------
    def button_callback(self, data):
        """
        Telemetrix callback for digital input:
        [pin_type, pin_number, pin_value, raw_timestamp]

        INPUT_PULLUP:
        released = 1
        pressed  = 0
        """
        try:
            pin_number = data[1]
            pin_value = data[2]
        except Exception:
            logging.warning(f"Unexpected callback format: {data}")
            return

        if pin_number not in self.btn_state:
            return

        previous_pin_value = self.btn_state[pin_number]

        # Keep current physical state
        self.btn_state[pin_number] = pin_value

        # Count only NEW press events: released=1 -> pressed=0
        if previous_pin_value == 1 and pin_value == 0:
            self.process_button_press(pin_number)

    # --------------------------------------------------------
    # Stepper callback
    # --------------------------------------------------------
    def stepper_completion_callback(self, data):
        self.stepper_done = True
        logging.info(f"Stepper movement completed. Callback data: {data}")

    # --------------------------------------------------------
    # Stepper test
    # --------------------------------------------------------
    def test_stepper(self):
        logging.info("=== STARTUP STEPPER TEST ===")
        logging.info(f"Trying to move the stepper by {STEPPER_TEST_STEPS} steps...")

        self.stepper_done = False

        self.board.stepper_move(self.motor_id, STEPPER_TEST_STEPS)
        self.board.stepper_run(
            self.motor_id,
            completion_callback=self.stepper_completion_callback
        )

        start_wait = time.perf_counter()

        while (
            not self.stepper_done
            and (time.perf_counter() - start_wait) < STEPPER_TIMEOUT_SEC
        ):
            time.sleep(0.02)

        if self.stepper_done:
            logging.info("Stepper startup test completed successfully.")
        else:
            logging.warning(
                "Stepper startup test failed or timed out. Motor did not visibly move."
            )

        # Turn off stepper coils after startup test
        self.stepper_power_off()

    # --------------------------------------------------------
    # Immediate reward delivery
    # --------------------------------------------------------
    def dispense_food_immediate(self, target_side):
        """
        Immediate reward delivery used for no correct press cases.
        """
        self.reward_delivery_timestamp = self.current_timestamp()

        logging.info(
            f"Dispensing immediate food to {target_side}... | "
            f"Reward_Delivery_Timestamp = {self.reward_delivery_timestamp}"
        )

        target_angle = self.servo_angle_for_side(target_side)

        self.board.servo_write(PIN_SERVO, target_angle)
        time.sleep(0.25)

        self.run_stepper_once()

        self.board.servo_write(PIN_SERVO, ANGLE_CENTER)
        time.sleep(0.20)

    # --------------------------------------------------------
    # CSV helpers
    # --------------------------------------------------------
    def create_session_file(self, session_id, macaque_to_test, wall_present):
        wall_status = "ON" if wall_present else "OFF"

        filename = (
            f"Data_{session_id}_"
            f"{macaque_to_test}_"
            f"WALL_{wall_status}_"
            f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )

        self.current_csv_path = self.data_dir / filename

        with self.current_csv_path.open(
            "w",
            newline="",
            encoding="utf-8"
        ) as file:
            writer = csv.writer(file)
            writer.writerow([
                "Trial_Start_Timestamp",
                "Stimulus_ON_Timestamp",
                "Button_Press_Timestamp",
                "Reward_Delivery_Timestamp",
                "Session_ID",
                "Trial_Num",
                "Active_Side",
                "Wall_Present",
                "ITI_sec",
                "Premature_ITI_Presses",
                "Pressed_Correct",
                "Wrong_Button_Pressed",
                "Omission",
                "Latency_ms",
                "Reward_Recipient",
                "Servo_Delay_sec",
                "Stepper_Delay_sec",
                "Servo_Hold_sec"
            ])
            file.flush()
            os.fsync(file.fileno())

        logging.info(f"CSV created: {self.current_csv_path}")

    def append_trial_to_csv(
        self,
        session_id,
        trial_num,
        wall_present,
        iti_sec,
        reward_recipient
    ):
        if self.current_csv_path is None:
            raise RuntimeError("CSV path is not initialized. create_session_file() was not called.")

        row = [
            self.trial_start_timestamp,
            self.stimulus_on_timestamp,
            self.button_press_timestamp,
            self.reward_delivery_timestamp,
            session_id,
            trial_num,
            self.active_side,
            wall_present,
            iti_sec,
            self.premature_iti_presses,
            1 if self.button_pressed else 0,
            1 if self.wrong_button_pressed else 0,
            1 if self.omission else 0,
            self.reaction_time_ms,
            reward_recipient,
            self.current_servo_delay_sec,
            self.current_stepper_delay_sec,
            self.current_servo_hold_sec
        ]

        with self.current_csv_path.open(
            "a",
            newline="",
            encoding="utf-8"
        ) as file:
            writer = csv.writer(file)
            writer.writerow(row)

            # Force immediate write to disk
            file.flush()
            os.fsync(file.fileno())

        logging.info(
            f"CSV row appended and flushed | "
            f"Trial={trial_num} | "
            f"CSV={self.current_csv_path}"
        )

    # --------------------------------------------------------
    # Main session logic
    # --------------------------------------------------------
    def run_session(self, session_id, macaque_to_test="M1", wall_present=False):
        logging.info("")
        logging.info(f"=== STARTING SESSION: {session_id} ===")
        logging.info(
            f"Target side: {macaque_to_test} | "
            f"Wall present: {wall_present}"
        )

        if macaque_to_test not in ("M1", "M2"):
            raise ValueError("macaque_to_test must be 'M1' or 'M2'")

        self.create_session_file(
            session_id=session_id,
            macaque_to_test=macaque_to_test,
            wall_present=wall_present
        )

        logging.info(f"CSV absolute path: {self.current_csv_path.resolve()}")
        logging.info(f"CSV exists after creation: {self.current_csv_path.exists()}")

        session_start = time.time()

        for trial in range(1, NUM_TRIALS + 1):
            if (time.time() - session_start) > SESSION_DURATION_SEC:
                logging.warning("Session duration limit reached.")
                break

            trial_saved = False
            iti_sec = None
            reward_recipient = None

            try:
                # ----------------------------
                # ITI
                # ----------------------------
                iti_sec = random.randint(MIN_ITI, MAX_ITI)
                self.premature_iti_presses = 0
                self.in_iti = True

                logging.info(f"--- Trial {trial}/{NUM_TRIALS} ---")
                logging.info(f"ITI = {iti_sec} s")

                self.safe_sleep(iti_sec)

                self.in_iti = False

                logging.info(
                    f"ITI complete | "
                    f"Premature_ITI_Presses = {self.premature_iti_presses}"
                )

                # ----------------------------
                # Trial init
                # ----------------------------
                self.active_side = macaque_to_test
                self.correct_button_pin = self.button_pin_for_side(self.active_side)
                self.correct_led_pin = self.led_pin_for_side(self.active_side)

                self.button_pressed = False
                self.wrong_button_pressed = False
                self.omission = False
                self.pressed_pin = None
                self.reaction_time_ms = int(TRIAL_DURATION_SEC * 1000)

                self.trial_start_timestamp = None
                self.stimulus_on_timestamp = None
                self.button_press_timestamp = None
                self.reward_delivery_timestamp = None

                self.stimulus_buzzer_active = False
                self.stimulus_buzzer_end_time_perf = None

                self.correct_press_buzzer_active = False
                self.correct_press_buzzer_end_time_perf = None
                self.correct_press_buzzer_pending = False
                self.correct_press_buzzer_start_time_perf = None

                self.reset_scheduled_reward_state()

                self.trial_start_perf = time.perf_counter()
                self.in_trial = True

                # Activate visual stimulus + short buzzer cue at trial onset
                self.trigger_stimulus_cue(self.active_side)

                self.stimulus_on_timestamp = self.current_timestamp()
                self.trial_start_timestamp = self.stimulus_on_timestamp

                logging.info(
                    f"Stimulus ON. Tested side = {self.active_side} | "
                    f"LED side = {self.active_side} | "
                    f"Correct button pin = {self.correct_button_pin} | "
                    f"Stimulus_ON_Timestamp = {self.stimulus_on_timestamp}"
                )

                # Trial loop
                trial_deadline = time.perf_counter() + TRIAL_DURATION_SEC

                while time.perf_counter() < trial_deadline:
                    self.update_scheduled_reward()
                    self.update_stimulus_buzzer()
                    self.update_correct_press_buzzer()
                    time.sleep(TRIAL_POLL_INTERVAL_SEC)

                self.in_trial = False
                self.leds_off()

                # Continue until scheduled reward and buzzers are finished
                while (
                    self.reward_schedule_active
                    or self.stimulus_buzzer_active
                    or self.correct_press_buzzer_active
                    or self.correct_press_buzzer_pending
                ):
                    self.update_scheduled_reward()
                    self.update_stimulus_buzzer()
                    self.update_correct_press_buzzer()
                    time.sleep(TRIAL_POLL_INTERVAL_SEC)

                # Omission = no correct and no wrong button press during the trial
                self.omission = (
                    not self.button_pressed
                    and not self.wrong_button_pressed
                )

                # Reward logic
                reward_recipient = self.get_reward_recipient()

                logging.info(
                    f"Reward decision | "
                    f"tested_macaque={self.active_side} | "
                    f"button_pressed={self.button_pressed} | "
                    f"wrong_button_pressed={self.wrong_button_pressed} | "
                    f"omission={self.omission} | "
                    f"reward_recipient={reward_recipient}"
                )

                # Only deliver immediate reward if there was NO correct press
                if not self.button_pressed:
                    self.dispense_food_immediate(reward_recipient)

            except KeyboardInterrupt:
                logging.warning(f"Trial {trial} interrupted by user Ctrl+C.")

                self.in_trial = False
                self.in_iti = False

                try:
                    self.leds_off()
                except Exception:
                    pass

                self.omission = (
                    not self.button_pressed
                    and not self.wrong_button_pressed
                )

                if reward_recipient is None and self.active_side is not None:
                    reward_recipient = self.get_reward_recipient()

                # The finally block below will save the current trial before exit.
                raise

            except Exception as e:
                logging.exception(f"Error during trial {trial}: {e}")

                self.in_trial = False
                self.in_iti = False

                try:
                    self.leds_off()
                except Exception:
                    pass

                self.omission = (
                    not self.button_pressed
                    and not self.wrong_button_pressed
                )

                if reward_recipient is None and self.active_side is not None:
                    reward_recipient = self.get_reward_recipient()

            finally:
                if reward_recipient is None:
                    reward_recipient = "UNKNOWN"

                if iti_sec is None:
                    iti_sec = ""

                try:
                    self.append_trial_to_csv(
                        session_id=session_id,
                        trial_num=trial,
                        wall_present=wall_present,
                        iti_sec=iti_sec,
                        reward_recipient=reward_recipient
                    )

                    trial_saved = True

                    logging.info(
                        f"Trial {trial} saved | "
                        f"Tested side: {self.active_side} | "
                        f"Correct press: {self.button_pressed} | "
                        f"Wrong press: {self.wrong_button_pressed} | "
                        f"Omission: {self.omission} | "
                        f"Premature ITI presses: {self.premature_iti_presses} | "
                        f"Latency: {self.reaction_time_ms} ms | "
                        f"Stimulus_ON_Timestamp: {self.stimulus_on_timestamp} | "
                        f"Button_Press_Timestamp: {self.button_press_timestamp} | "
                        f"Reward_Delivery_Timestamp: {self.reward_delivery_timestamp} | "
                        f"Reward given to: {reward_recipient} | "
                        f"Servo_Delay_sec: {self.current_servo_delay_sec} | "
                        f"Stepper_Delay_sec: {self.current_stepper_delay_sec} | "
                        f"Servo_Hold_sec: {self.current_servo_hold_sec}"
                    )

                    logging.info(f"CSV exists after append: {self.current_csv_path.exists()}")
                    logging.info(f"CSV size after append: {self.current_csv_path.stat().st_size} bytes")

                except Exception as e:
                    logging.exception(f"CRITICAL: could not save trial {trial} to CSV: {e}")

            if not trial_saved:
                logging.warning(f"Trial {trial} was not saved correctly.")

        logging.info("=== SESSION COMPLETE ===")

    # --------------------------------------------------------
    # Shutdown
    # --------------------------------------------------------
    def shutdown(self):
        logging.info("Shutting down hardware safely...")

        try:
            self.leds_off()
            self.board.digital_write(PIN_BUZZER, 0)
            self.stepper_power_off()
            self.board.servo_write(PIN_SERVO, ANGLE_CENTER)
            time.sleep(0.3)
            self.board.shutdown()

        except Exception as e:
            logging.warning(f"Ignored shutdown exception: {e}")


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    data_directory = setup_environment()
    experiment = None

    try:
        macaque_to_test = ask_macaque()
        wall_present = ask_wall_present()
        session_number = ask_session_number()

        session_id = f"SESS_{session_number:03d}"

        logging.info(
            f"Operator inputs | "
            f"macaque_to_test={macaque_to_test} | "
            f"wall_present={wall_present} | "
            f"session_number={session_number} | "
            f"session_id={session_id}"
        )

        experiment = MacaqueExperiment(
            data_dir=data_directory,
            port=None
        )

        experiment.run_session(
            session_id=session_id,
            macaque_to_test=macaque_to_test,
            wall_present=wall_present
        )

    except KeyboardInterrupt:
        logging.warning("Experiment interrupted by user (Ctrl+C).")

    except Exception as e:
        logging.exception(f"Fatal error: {e}")

    finally:
        if experiment is not None:
            experiment.shutdown()

        input("\nPress Enter to close the program...")
