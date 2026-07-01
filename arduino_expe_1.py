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
MAX_ITI = 15

# ----------------------------
# Hardware pins
# ----------------------------
PIN_SERVO = 3
PIN_BTN_M1 = 4
PIN_BTN_M2 = 5
PIN_LED_M1 = 6
PIN_LED_M2 = 7
PIN_STEPPER = [8, 9, 10, 11]  # IN1, IN2, IN3, IN4

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
STEPS_90_DEG = 1024              # Quarter turn
STEPPER_TEST_STEPS = 128         # Small visible test at startup
STEPPER_MAX_SPEED = 180          # Reduced speed for reliability
STEPPER_ACCELERATION = 80        # Reduced acceleration for reliability
STEPPER_TIMEOUT_SEC = 8.0

RUN_STEPPER_TEST_AT_STARTUP = True


# ============================================================
# LOGGING / ENVIRONMENT
# ============================================================

def setup_environment():
    """
    Creates the data folder on Desktop (or Bureau) and configures logging.
    """
    home = Path.home()
    desktop = home / "Desktop"
    if not desktop.exists():
        desktop = home / "Bureau"

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
# MAIN CLASS
# ============================================================

class MacaqueExperiment:
    def __init__(self, data_dir, port=None):
        self.data_dir = Path(data_dir)

        logging.info("Initializing experiment and connecting to board...")

        try:
            self.board = telemetrix.Telemetrix(com_port=port, arduino_wait=8)
        except Exception as e:
            logging.exception(f"Unable to connect to Arduino/Telemetrix: {e}")
            raise

        # Trial/session state
        self.in_trial = False
        self.in_iti = False
        self.active_side = None              # "M1" or "M2"
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

        self.setup_hardware()

    # --------------------------------------------------------
    # Timestamp helper
    # --------------------------------------------------------
    def current_timestamp(self):
        return datetime.now().isoformat(sep=" ", timespec="milliseconds")

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
        - If the tested macaque presses the correct button, reward goes to the tested macaque.
        - If the tested macaque does not press the correct button, reward goes to the other macaque.
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
        self.board.set_pin_mode_digital_input_pullup(PIN_BTN_M1, callback=self.button_callback)
        time.sleep(0.05)
        self.board.set_pin_mode_digital_input_pullup(PIN_BTN_M2, callback=self.button_callback)
        time.sleep(1.0)

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
    # Button event processing
    # --------------------------------------------------------
    def process_button_press(self, pin_number):
        """
        Processes a confirmed press transition on a button.
        This function is called only when the input changes from released=1 to pressed=0.
        """
        button_event_timestamp = self.current_timestamp()

        logging.info(
            f"Button press transition detected on pin {pin_number} | "
            f"in_iti={self.in_iti} | in_trial={self.in_trial} | "
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

        # Outside trial and outside ITI => ignore for experimental scoring
        if not self.in_trial:
            return

        # Store timestamp of the first button press during the trial
        if self.button_press_timestamp is None:
            self.button_press_timestamp = button_event_timestamp

        # Correct button = same side as the tested macaque / lit LED
        if pin_number == self.correct_button_pin and not self.button_pressed:
            self.reaction_time_ms = int((time.perf_counter() - self.trial_start_perf) * 1000)
            self.button_pressed = True
            self.pressed_pin = pin_number

            logging.info(
                f"[{self.active_side}] Correct button press detected "
                f"on pin {pin_number} at {self.reaction_time_ms} ms | "
                f"Button_Press_Timestamp = {button_event_timestamp}"
            )

            # Immediate visual feedback
            self.board.digital_write(self.correct_led_pin, 0)

        # Wrong button = ignored for reward, but logged
        elif pin_number != self.correct_button_pin:
            self.wrong_button_pressed = True

            logging.info(
                f"[{self.active_side}] Wrong button pressed on pin {pin_number} | "
                f"Expected correct pin = {self.correct_button_pin} | "
                f"Button_Press_Timestamp = {button_event_timestamp} (ignored for reward)."
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
        self.board.stepper_run(self.motor_id, completion_callback=self.stepper_completion_callback)

        start_wait = time.perf_counter()
        while not self.stepper_done and (time.perf_counter() - start_wait) < STEPPER_TIMEOUT_SEC:
            time.sleep(0.02)

        if self.stepper_done:
            logging.info("Stepper startup test completed successfully.")
        else:
            logging.warning("Stepper startup test failed or timed out. Motor did not visibly move.")

    # --------------------------------------------------------
    # Reward delivery
    # --------------------------------------------------------
    def dispense_food(self, target_side):
        self.reward_delivery_timestamp = self.current_timestamp()

        logging.info(
            f"Dispensing food to {target_side}... | "
            f"Reward_Delivery_Timestamp = {self.reward_delivery_timestamp}"
        )

        # 1) Point servo to target side
        target_angle = self.servo_angle_for_side(target_side)
        self.board.servo_write(PIN_SERVO, target_angle)
        time.sleep(0.8)

        # 2) Stepper quarter turn
        self.stepper_done = False
        self.board.stepper_move(self.motor_id, STEPS_90_DEG)
        self.board.stepper_run(self.motor_id, completion_callback=self.stepper_completion_callback)

        start_wait = time.perf_counter()
        while not self.stepper_done and (time.perf_counter() - start_wait) < STEPPER_TIMEOUT_SEC:
            time.sleep(0.02)

        if not self.stepper_done:
            logging.warning("Stepper timeout reached before completion callback.")

        # 3) Return servo to center
        self.board.servo_write(PIN_SERVO, ANGLE_CENTER)
        time.sleep(0.5)

    # --------------------------------------------------------
    # CSV helpers
    # --------------------------------------------------------
    def create_session_file(self, session_id):
        filename = f"Data_{session_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.current_csv_path = self.data_dir / filename

        with self.current_csv_path.open("w", newline="", encoding="utf-8") as file:
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
                "Reward_Recipient"
            ])

        logging.info(f"CSV created: {self.current_csv_path}")

    def append_trial_to_csv(self, session_id, trial_num, wall_present, iti_sec, reward_recipient):
        with self.current_csv_path.open("a", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow([
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
                reward_recipient
            ])

    # --------------------------------------------------------
    # Main session logic
    # --------------------------------------------------------
    def run_session(self, session_id, macaque_to_test="M1", wall_present=False):
        logging.info("")
        logging.info(f"=== STARTING SESSION: {session_id} ===")
        logging.info(f"Target side: {macaque_to_test} | Wall present: {wall_present}")

        if macaque_to_test not in ("M1", "M2"):
            raise ValueError("macaque_to_test must be 'M1' or 'M2'")

        self.create_session_file(session_id)

        session_start = time.time()

        for trial in range(1, NUM_TRIALS + 1):
            if (time.time() - session_start) > SESSION_DURATION_SEC:
                logging.warning("Session duration limit reached.")
                break

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
                f"ITI complete | Premature_ITI_Presses = {self.premature_iti_presses}"
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

            self.trial_start_perf = time.perf_counter()
            self.in_trial = True

            # Light the LED for the same side whose button must be pressed
            self.light_only_led_for_side(self.active_side)

            self.stimulus_on_timestamp = self.current_timestamp()
            self.trial_start_timestamp = self.stimulus_on_timestamp

            logging.info(
                f"Stimulus ON. Tested side = {self.active_side} | "
                f"LED side = {self.active_side} | "
                f"Correct button pin = {self.correct_button_pin} | "
                f"Stimulus_ON_Timestamp = {self.stimulus_on_timestamp}"
            )

            # Mandatory 10-second window
            self.safe_sleep(TRIAL_DURATION_SEC)

            # End of trial
            self.in_trial = False
            self.leds_off()

            # Omission = no correct and no wrong button press during the trial
            self.omission = not self.button_pressed and not self.wrong_button_pressed

            # ----------------------------
            # Reward logic
            # Rule:
            # - Correct press by tested macaque => reward tested macaque.
            # - No correct press => reward other macaque.
            # ----------------------------
            reward_recipient = self.get_reward_recipient()

            logging.info(
                f"Reward decision | tested_macaque={self.active_side} | "
                f"button_pressed={self.button_pressed} | "
                f"wrong_button_pressed={self.wrong_button_pressed} | "
                f"omission={self.omission} | "
                f"reward_recipient={reward_recipient}"
            )

            self.dispense_food(reward_recipient)

            # Save data
            self.append_trial_to_csv(
                session_id=session_id,
                trial_num=trial,
                wall_present=wall_present,
                iti_sec=iti_sec,
                reward_recipient=reward_recipient
            )

            logging.info(
                f"Trial {trial} saved | Tested side: {self.active_side} | "
                f"Correct press: {self.button_pressed} | "
                f"Wrong press: {self.wrong_button_pressed} | "
                f"Omission: {self.omission} | "
                f"Premature ITI presses: {self.premature_iti_presses} | "
                f"Latency: {self.reaction_time_ms} ms | "
                f"Stimulus_ON_Timestamp: {self.stimulus_on_timestamp} | "
                f"Button_Press_Timestamp: {self.button_press_timestamp} | "
                f"Reward_Delivery_Timestamp: {self.reward_delivery_timestamp} | "
                f"Reward given to: {reward_recipient}"
            )

        logging.info("=== SESSION COMPLETE ===")

    # --------------------------------------------------------
    # Shutdown
    # --------------------------------------------------------
    def shutdown(self):
        logging.info("Shutting down hardware safely...")
        try:
            self.leds_off()
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
        experiment = MacaqueExperiment(data_dir=data_directory, port=None)

        experiment.run_session(
            session_id="SESS_001",
            macaque_to_test="M1",   # Change to "M2" to test M2
            wall_present=False
        )

    except KeyboardInterrupt:
        logging.warning("Experiment interrupted by user (Ctrl+C).")
    except Exception as e:
        logging.exception(f"Fatal error: {e}")
    finally:
        if experiment is not None:
            experiment.shutdown()

        input("\nPress Enter to close the program...")
