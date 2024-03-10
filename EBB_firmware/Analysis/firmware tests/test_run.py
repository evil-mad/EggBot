# Main EBB hardware in the loop test script
#
# Called without any parameters, this script will run all EBB hardware tests
# A single sub test can be run by naming that test as the first parameter
#


from scripts.test_test import test_test_run

print("Run test: test_test_run()")
test_test_run()
print("Complete")