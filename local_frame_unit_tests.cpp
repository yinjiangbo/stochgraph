// Unit tests for LocalFrame.
#include "local_frame.hpp"

#include <gtest/gtest.h>

TEST(LocalFrameUnitTests, TestNothing) {

   LocalFrame* instance = LocalFrame::GetInstance();

   // TODO These values should be read from a data file.
   double offset[] = {-672139.48,-3972908.10};
   instance->AddOffset(offset[0], offset[1]);

   double x, y;
   instance->LongLatToLocal(-78.910, 35.907, &x, &y);

   std::cout << x << " " << y << " are projected points.\n";

   EXPECT_TRUE(true);
}


int main(int argc, char** argv) {
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
