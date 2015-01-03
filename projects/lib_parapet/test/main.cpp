//============================================================================
// Execution entry-point for Parapet unit tests
//============================================================================

#include "gtest/gtest.h"

int main(int argc, char **argv) 
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

