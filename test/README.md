# Test directory
This directory contains all test source files which test the correctness of 
X-View. All tests are implemented as G-tests.

### Test example
Let's say we want to test a module of X-View called _"testable"_. The 
structure adopted by the implemented tests in this folder is the following:

1) a _"main"_ file called `test_testable_main.cc` of the form:
   ```lang:cpp
    #include <gtest/gtest.h>
  
    #include "test_testable.h"
  
    #include <x_view_core/...h>
  
    #include <glog/logging.h>
  
    using namespace x_view_test;
  
    TEST(XViewSlamTestSuite, test_testable) {
  
      LOG(INFO) << "\n\n====Testing testable====";
  
      // Prepare dataset or other structures used by testable
      ...
      
      // Test testable in different ways.
      testTestable1();
      testTestable2();
      ...
  
    }
    ```

2) a header file called `test_testable.h` containing all the function/class 
declarations used by the test of the form:
   ```lang:cpp
   #ifndef X_VIEW_TEST_TESTABLE_H
   #define X_VIEW_TEST_TESTABLE_H
   
   
   namespace x_view_test {
   
   void testTestable1();   
   void testTestable2();
   ...
   
   }
   
   #endif //X_VIEW_TEST_TESTABLE_H

   ```

3) a source file called `test_testable.cc` containing the test implementation.
