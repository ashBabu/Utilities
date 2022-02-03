import math


def add_patch():
    def log100(inp):
        return math.log(inp, 100)
    math.log100 = log100

    # Write the code that goes here
    # pass

# print(math.log(10, 100))
# Example case.
add_patch()
print(math.log100(10))