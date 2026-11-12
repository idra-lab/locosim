from numpy.testing import assert_almost_equal
from termcolor import colored

class UnitTest:
    def __init__(self):
        self.errors = []

    def check_assertion(self, value, expected, decimal, description):
        try:
            assert_almost_equal(value, expected, decimal=decimal)
        except AssertionError as e:
            self.errors.append(f"{description}: {e}")

    def report(self):
        if self.errors:
            print(colored("Unit test failed:", "red"))
            for error in self.errors:
                print(error)
        else:
            print(colored("Unit test succesful!", "green"))
