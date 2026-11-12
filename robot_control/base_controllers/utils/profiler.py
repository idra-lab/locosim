from line_profiler import LineProfiler
from termcolor import colored
import io

class Profiler:

    def __init__(self, function_name=None):
        self.lp = LineProfiler()
        if function_name is None:
            print(colored("Need to define a function to profile!","red"))
        self.lp.add_function(function_name)  # Profile this function
        self.lp.enable()

    def get_total_time(self):
        self.lp.disable()
        # Capture output and extract average time
        output = io.StringIO()
        self.lp.print_stats(stream=output)
        lines = output.getvalue().split("\n")
        print(colored("Profiling simulateOnestep", "red"))
        for line in lines:
            if "Total time:" in line:
                total_time = float(line.split()[2])  # Extract the total execution time
                break
        return total_time