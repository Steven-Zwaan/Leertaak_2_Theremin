#include <unity.h>
#include <stdint.h>

// Mock mapping implementation for testing
static uint16_t map_distance_to_freq(uint16_t distance)
{
    const uint16_t fmin = 230;
    const uint16_t fmax = 1400;
    const uint16_t distMax = 65;

    // Invalid distance check
    if (distance > distMax)
    {
        return fmin;
    }

    // Linear mapping: f = fmin + (fmax - fmin) * (distMax - d) / distMax
    // Closer distance = higher frequency
    uint32_t range = fmax - fmin;
    uint32_t inverted = distMax - distance;
    uint32_t freq = fmin + (range * inverted) / distMax;

    // Clamp to valid range
    if (freq < fmin)
        freq = fmin;
    if (freq > fmax)
        freq = fmax;

    return (uint16_t)freq;
}

// Unity test setup/teardown
void setUp(void)
{
    // Setup before each test
}

void tearDown(void)
{
    // Cleanup after each test
}

// Test cases

void test_mapping_minimum_distance(void)
{
    // At distance 0, should return maximum frequency
    uint16_t freq = map_distance_to_freq(0);
    TEST_ASSERT_EQUAL_UINT16(1400, freq);
}

void test_mapping_maximum_distance(void)
{
    // At distance 65, should return minimum frequency
    uint16_t freq = map_distance_to_freq(65);
    TEST_ASSERT_EQUAL_UINT16(230, freq);
}

void test_mapping_mid_distance(void)
{
    // At distance 32-33 (roughly middle), should be near middle frequency
    uint16_t freq = map_distance_to_freq(32);

    // Middle frequency = (230 + 1400) / 2 = 815
    // Allow some tolerance due to integer division
    TEST_ASSERT_GREATER_OR_EQUAL(800, freq);
    TEST_ASSERT_LESS_OR_EQUAL(830, freq);
}

void test_mapping_out_of_range_high(void)
{
    // Distance > 65 should return minimum frequency
    uint16_t freq = map_distance_to_freq(100);
    TEST_ASSERT_EQUAL_UINT16(230, freq);
}

void test_mapping_out_of_range_very_high(void)
{
    // Very large distance should still return minimum frequency
    uint16_t freq = map_distance_to_freq(1000);
    TEST_ASSERT_EQUAL_UINT16(230, freq);
}

void test_mapping_close_distance(void)
{
    // Very close distance (5cm) should give high frequency
    uint16_t freq = map_distance_to_freq(5);

    // Expected: 230 + 1170 * (65-5)/65 = 230 + 1170 * 60/65 ≈ 1310
    TEST_ASSERT_GREATER_OR_EQUAL(1300, freq);
    TEST_ASSERT_LESS_OR_EQUAL(1320, freq);
}

void test_mapping_far_distance(void)
{
    // Far distance (60cm) should give low frequency
    uint16_t freq = map_distance_to_freq(60);

    // Expected: 230 + 1170 * (65-60)/65 = 230 + 1170 * 5/65 ≈ 320
    TEST_ASSERT_GREATER_OR_EQUAL(310, freq);
    TEST_ASSERT_LESS_OR_EQUAL(330, freq);
}

void test_mapping_linearity(void)
{
    // Test that frequency decreases linearly with distance
    uint16_t freq10 = map_distance_to_freq(10);
    uint16_t freq20 = map_distance_to_freq(20);
    uint16_t freq30 = map_distance_to_freq(30);

    // Each 10cm should result in similar frequency drop
    uint16_t delta1 = freq10 - freq20;
    uint16_t delta2 = freq20 - freq30;

    // Deltas should be roughly equal (within 10% tolerance)
    int16_t diff = delta1 - delta2;
    TEST_ASSERT_TRUE(diff >= -20 && diff <= 20);
}

void test_mapping_monotonic_decrease(void)
{
    // Frequency should monotonically decrease with increasing distance
    uint16_t prev_freq = map_distance_to_freq(0);

    for (uint16_t dist = 1; dist <= 65; dist++)
    {
        uint16_t curr_freq = map_distance_to_freq(dist);
        TEST_ASSERT_LESS_OR_EQUAL(prev_freq, curr_freq);
        prev_freq = curr_freq;
    }
}

void test_mapping_boundary_64(void)
{
    // Test boundary at 64cm (just within range)
    uint16_t freq = map_distance_to_freq(64);

    // Should be just above minimum
    TEST_ASSERT_GREATER_THAN(230, freq);
    TEST_ASSERT_LESS_THAN(250, freq);
}

void test_mapping_boundary_66(void)
{
    // Test boundary at 66cm (just out of range)
    uint16_t freq = map_distance_to_freq(66);
    TEST_ASSERT_EQUAL_UINT16(230, freq);
}

void test_mapping_step_size(void)
{
    // Test that 1cm steps produce reasonable frequency changes
    for (uint16_t dist = 0; dist < 65; dist++)
    {
        uint16_t freq1 = map_distance_to_freq(dist);
        uint16_t freq2 = map_distance_to_freq(dist + 1);

        uint16_t diff = freq1 - freq2;

        // Frequency difference per cm should be reasonable
        // Total range 1170Hz over 65cm = ~18Hz per cm
        TEST_ASSERT_GREATER_OR_EQUAL(10, diff);
        TEST_ASSERT_LESS_OR_EQUAL(25, diff);
    }
}

void test_mapping_musical_range(void)
{
    // Test that the frequency range covers useful musical range
    // A3 = 220Hz, A5 = 880Hz, A6 = 1760Hz

    uint16_t freq_min = map_distance_to_freq(65);
    uint16_t freq_max = map_distance_to_freq(0);

    // Should cover more than 2 octaves
    // 2 octaves = factor of 4, 3 octaves = factor of 8
    float ratio = (float)freq_max / (float)freq_min;

    TEST_ASSERT_GREATER_THAN(4.0f, ratio); // More than 2 octaves
}

void test_mapping_specific_notes(void)
{
    // Test that we can reach common musical notes
    // A3 = 220Hz, C4 = 262Hz, A4 = 440Hz, C5 = 523Hz, A5 = 880Hz

    uint16_t found_440 = 0;
    uint16_t found_880 = 0;

    for (uint16_t dist = 0; dist <= 65; dist++)
    {
        uint16_t freq = map_distance_to_freq(dist);

        // Check if we're close to A4 (440Hz)
        if (freq >= 430 && freq <= 450)
        {
            found_440 = 1;
        }

        // Check if we're close to A5 (880Hz)
        if (freq >= 870 && freq <= 890)
        {
            found_880 = 1;
        }
    }

    TEST_ASSERT_TRUE(found_440);
    TEST_ASSERT_TRUE(found_880);
}

void test_mapping_precision(void)
{
    // Test that we maintain precision throughout the range
    for (uint16_t dist = 0; dist <= 65; dist += 5)
    {
        uint16_t freq = map_distance_to_freq(dist);

        // Frequency should always be in valid range
        TEST_ASSERT_GREATER_OR_EQUAL(230, freq);
        TEST_ASSERT_LESS_OR_EQUAL(1400, freq);
    }
}

void test_mapping_overflow_protection(void)
{
    // Test with maximum uint16_t value
    uint16_t freq = map_distance_to_freq(65535);
    TEST_ASSERT_EQUAL_UINT16(230, freq);
}

// Main test runner
int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_mapping_minimum_distance);
    RUN_TEST(test_mapping_maximum_distance);
    RUN_TEST(test_mapping_mid_distance);
    RUN_TEST(test_mapping_out_of_range_high);
    RUN_TEST(test_mapping_out_of_range_very_high);
    RUN_TEST(test_mapping_close_distance);
    RUN_TEST(test_mapping_far_distance);
    RUN_TEST(test_mapping_linearity);
    RUN_TEST(test_mapping_monotonic_decrease);
    RUN_TEST(test_mapping_boundary_64);
    RUN_TEST(test_mapping_boundary_66);
    RUN_TEST(test_mapping_step_size);
    RUN_TEST(test_mapping_musical_range);
    RUN_TEST(test_mapping_specific_notes);
    RUN_TEST(test_mapping_precision);
    RUN_TEST(test_mapping_overflow_protection);

    return UNITY_END();
}
