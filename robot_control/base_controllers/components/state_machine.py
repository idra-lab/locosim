
import rospy
from base_controllers.utils.common_functions import checkRosMaster

class Timer:
    """Simple timer utility similar to force_timer_ in your C++ code."""
    def __init__(self, duration=0.0):
        self.start_time = None
        self.duration = duration
        self.active = False

    def start(self, start_time):
        """Start or restart the timer using an external time reference."""
        self.start_time = start_time
        self.active = True

    def reset(self):
        self.active = False
        self.start_time = None

    def is_elapsed(self, current_time):
        """Check if timer has elapsed based on provided time."""
        if not self.active or self.start_time is None:
            return False
        return (current_time - self.start_time) >= self.duration

    def get_elapsed_time(self, current_time):
        """Check if timer has elapsed based on provided time."""
        if not self.active or self.start_time is None:
            return False
        return (current_time - self.start_time)

class StateMachine:
    def __init__(self, verbose=1):
        self.states = []  # list of (name, action_fn, duration)
        self.current_index = 0
        self.timer = Timer()
        self.verbose = verbose
        self.running = False
        self.first_time = True  # flag passed to each state function
        self.booked_set_first_time = False

    def add_state(self, name, action_fn, duration = 1.):
        """Add a state with a name, a callable (action_fn), and a duration (seconds)."""
        self.states.append((name, action_fn, duration))

    def get_index(self, state_name):
        """Return the index of a given state name, or None if not found."""
        for i, (name, _, _) in enumerate(self.states):
            if name == state_name:
                return i
        return None

    def get_state_duration(self):
        """Return the current state's name (or None if stopped)."""
        if not self.running or not self.states:
            return None
        _, _, duration = self.states[self.current_index]
        return duration


    def start(self, start_time=0.0, start_state=None):
        """Start from a given or first state."""
        if not self.states:
            raise RuntimeError("No states defined.")
        if start_state is not None:
            idx = self.get_index(start_state)
            if idx is None:
                raise ValueError(f"State '{start_state}' not found.")
            self.current_index = idx
        else:
            self.current_index = 0
        name, _, duration = self.states[self.current_index]

        if self.verbose >= 1:
            print(f"\n→ Entering state: {name}")
        self.timer.duration = duration
        self.timer.start(start_time)
        self.running = True


    def _enter_state(self, start_time):
        name,  _, duration = self.states[self.current_index]
        if self.verbose >= 1:
            print(f"\n→ Entering state: {name}")
        self.timer.duration = duration
        self.timer.start(start_time)
        self.booked_set_first_time = True  # mark first iteration for this state

    def next(self, current_time=None):
        """Move to next state manually."""
        self.timer.reset()
        self.current_index += 1
        if self.current_index < len(self.states):
            self._enter_state(current_time)
        else:
            self.running = False
            print("\n✅ State machine finished all states.")

    def step(self, current_time):
        """Call periodically (e.g. in control loop)."""
        if not self.running:
            return

        # Execute current state's logic on every tick
        name, action_fn, _ = self.states[self.current_index]
        action_fn(self, current_time)
        if self.booked_set_first_time:
            self.first_time = True  # reset after the first call
            self.booked_set_first_time = False
        else:
            self.first_time = False  # reset after the first call

# --- # --- Example: Controller class using StateMachine ---
class QuadrupedSwing:
    def __init__(self):
        self.sm = StateMachine(verbose=1)
        # Bind state methods (these are instance methods)
        self.sm.add_state("start_crawl", self.start_crawl, 1.0)
        self.sm.add_state("unloadleg", self.unload_leg, 1.0)
        self.sm.add_state("swingleg", self.swing_leg, 1.0)

    # --- State functions ---
    def start_crawl(self, sm, t):
        if sm.first_time:
            print("Initializing crawl sequence...")
        print(f"Crawling...{t}")
        alpha = sm.timer.get_elapsed_time(t)/sm.get_state_duration()
        print(f"alpha:{alpha}")
        if sm.timer.is_elapsed(t):
            print("Crawl done → next state")
            sm.next(t)
        # event based
        # if event:
        #     print("Crawl done → next state")
        #     sm.next(t)

    def unload_leg(self,sm, t):
        if sm.first_time:
            print("Init unload leg...")
        print(f"Unloading leg...{t}")
        if sm.timer.is_elapsed(t):
            print("Leg unloaded → next state")
            sm.next(t)

    def swing_leg(self,sm, t):
        if sm.first_time:
            print("Init swing trajectory...")
        print(f"Swinging leg...{t}")
        if sm.timer.is_elapsed(t):
            print("Swing done → next state")
            sm.next(t)



if __name__ == "__main__":

    freq = 5  # Hz
    dt = 1. / freq
    time = 0.0

    quadrupedSwing = QuadrupedSwing()
    quadrupedSwing.sm.start(start_time=time)


    checkRosMaster()
    rospy.init_node("quadruped_crawl")
    rospy.loginfo("quadruped_crawl node initialized and running.")
    rate = rospy.Rate(freq)

    while not rospy.is_shutdown() and quadrupedSwing.sm.running:
        quadrupedSwing.sm.step(time)
        import numpy as np
        time = round(time + dt, 3)
        rate.sleep()
