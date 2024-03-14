# Main EBB hardware in the loop test script
#
# Called without any parameters, this script will run all EBB hardware tests
# A single sub test can be run by naming that test as the first parameter
#


from test_ISR_math import test_ISR_math_run


all_tests_pass = True

# Test : 
print("Run test: test_ISR_math_run()")
if test_ISR_math_run("..\\test input data\\test_inputs_simple.csv") == False:
    all_tests_pass = False

# Done running all tests
if all_tests_pass == True:
    print("All tests passed")
else:
    print("Some or all tests failed")

print("Complete")