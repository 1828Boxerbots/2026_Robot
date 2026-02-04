
#include "IntakeTest.h"

TEST(IntakeTest, DoesIntakeRun)
{
    IntakeSub::SetVelocity(2);

    EXPECT_DOUBLE_EQ
    (
        0.0,
        IntakeSub::GetVelocity();
    )
}
