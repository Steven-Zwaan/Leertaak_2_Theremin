#include <unity.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// Mock filter implementation for testing
#define MAX_FILTER_SIZE 15

typedef struct
{
    uint16_t value;
    uint8_t age;
} FilterEntry;

static FilterEntry buffer[MAX_FILTER_SIZE];
static uint8_t max_size = MAX_FILTER_SIZE;
static uint8_t current_count = 0;

// Helper function for sorting (qsort)
static int compare_uint16(const void *a, const void *b)
{
    return (*(uint16_t *)a - *(uint16_t *)b);
}

void filter_init(void)
{
    memset(buffer, 0, sizeof(buffer));
    current_count = 0;
}

void filter_add(uint16_t value)
{
    // Increment ages
    for (uint8_t i = 0; i < current_count; i++)
    {
        buffer[i].age++;
    }

    if (current_count < max_size)
    {
        // Add new entry
        buffer[current_count].value = value;
        buffer[current_count].age = 0;
        current_count++;
    }
    else
    {
        // Replace oldest entry
        uint8_t oldest_idx = 0;
        uint8_t oldest_age = buffer[0].age;
        for (uint8_t i = 1; i < max_size; i++)
        {
            if (buffer[i].age > oldest_age)
            {
                oldest_age = buffer[i].age;
                oldest_idx = i;
            }
        }
        buffer[oldest_idx].value = value;
        buffer[oldest_idx].age = 0;
    }
}

uint16_t filter_get_filtered(void)
{
    if (current_count == 0)
        return 0;

    // Copy values for sorting
    uint16_t temp[MAX_FILTER_SIZE];
    for (uint8_t i = 0; i < current_count; i++)
    {
        temp[i] = buffer[i].value;
    }

    // Sort using qsort
    qsort(temp, current_count, sizeof(uint16_t), compare_uint16);

    // Return median
    return temp[current_count / 2];
}

void filter_set_size(uint8_t size)
{
    if (size > 0 && size <= MAX_FILTER_SIZE)
    {
        max_size = size;
        if (current_count > max_size)
        {
            current_count = max_size;
        }
    }
}

uint8_t filter_get_size(void)
{
    return max_size;
}

// Unity test setup/teardown
void setUp(void)
{
    filter_init();
    filter_set_size(15);
}

void tearDown(void)
{
    // Clean up after each test
}

// Test cases

void test_filter_init(void)
{
    filter_init();
    TEST_ASSERT_EQUAL_UINT16(0, filter_get_filtered());
}

void test_filter_single_value(void)
{
    filter_add(100);
    TEST_ASSERT_EQUAL_UINT16(100, filter_get_filtered());
}

void test_filter_median_odd_count(void)
{
    filter_add(10);
    filter_add(30);
    filter_add(20);
    // Sorted: 10, 20, 30 -> median = 20
    TEST_ASSERT_EQUAL_UINT16(20, filter_get_filtered());
}

void test_filter_median_even_count(void)
{
    filter_add(10);
    filter_add(40);
    filter_add(20);
    filter_add(30);
    // Sorted: 10, 20, 30, 40 -> median = index 2 = 30
    TEST_ASSERT_EQUAL_UINT16(30, filter_get_filtered());
}

void test_filter_removes_outliers(void)
{
    filter_set_size(5);
    filter_add(100);
    filter_add(105);
    filter_add(102);
    filter_add(500); // Outlier
    filter_add(103);
    // Sorted: 100, 102, 103, 105, 500 -> median = 103
    TEST_ASSERT_EQUAL_UINT16(103, filter_get_filtered());
}

void test_filter_size_change(void)
{
    filter_set_size(3);
    TEST_ASSERT_EQUAL_UINT8(3, filter_get_size());

    filter_set_size(10);
    TEST_ASSERT_EQUAL_UINT8(10, filter_get_size());
}

void test_filter_invalid_size(void)
{
    filter_set_size(5);
    filter_set_size(0);                            // Invalid
    TEST_ASSERT_EQUAL_UINT8(5, filter_get_size()); // Should remain unchanged

    filter_set_size(20);                           // Too large
    TEST_ASSERT_EQUAL_UINT8(5, filter_get_size()); // Should remain unchanged
}

void test_filter_buffer_overflow(void)
{
    filter_set_size(3);

    // Fill buffer
    filter_add(10);
    filter_add(20);
    filter_add(30);

    // Add fourth value (should replace oldest)
    filter_add(40);

    // Should now have 20, 30, 40 -> median = 30
    TEST_ASSERT_EQUAL_UINT16(30, filter_get_filtered());
}

void test_filter_age_rotation(void)
{
    filter_set_size(3);

    // Add values in sequence
    filter_add(100); // age will become 2
    filter_add(200); // age will become 1
    filter_add(300); // age will be 0

    // Add fourth value - should replace oldest (100)
    filter_add(400);

    // Buffer should now have: 200, 300, 400
    // Sorted: 200, 300, 400 -> median = 300
    TEST_ASSERT_EQUAL_UINT16(300, filter_get_filtered());
}

void test_filter_consistent_values(void)
{
    filter_set_size(7);

    // Add same value multiple times
    for (int i = 0; i < 7; i++)
    {
        filter_add(250);
    }

    TEST_ASSERT_EQUAL_UINT16(250, filter_get_filtered());
}

void test_filter_ascending_sequence(void)
{
    filter_set_size(5);

    filter_add(10);
    filter_add(20);
    filter_add(30);
    filter_add(40);
    filter_add(50);

    // Median of 10, 20, 30, 40, 50 = 30
    TEST_ASSERT_EQUAL_UINT16(30, filter_get_filtered());
}

void test_filter_descending_sequence(void)
{
    filter_set_size(5);

    filter_add(50);
    filter_add(40);
    filter_add(30);
    filter_add(20);
    filter_add(10);

    // Sorted: 10, 20, 30, 40, 50 -> median = 30
    TEST_ASSERT_EQUAL_UINT16(30, filter_get_filtered());
}

void test_filter_noise_reduction(void)
{
    filter_set_size(9);

    // Simulated noisy signal around 200
    uint16_t noisy_values[] = {200, 205, 195, 202, 198, 201, 203, 199, 204};

    for (int i = 0; i < 9; i++)
    {
        filter_add(noisy_values[i]);
    }

    uint16_t result = filter_get_filtered();

    // Median should be close to 200
    TEST_ASSERT_GREATER_OR_EQUAL(198, result);
    TEST_ASSERT_LESS_OR_EQUAL(202, result);
}

void test_filter_spike_rejection(void)
{
    filter_set_size(7);

    // Add stable values with one spike
    filter_add(100);
    filter_add(102);
    filter_add(101);
    filter_add(999); // Spike
    filter_add(100);
    filter_add(103);
    filter_add(101);

    uint16_t result = filter_get_filtered();

    // Median should be around 101, not affected by spike
    TEST_ASSERT_GREATER_OR_EQUAL(100, result);
    TEST_ASSERT_LESS_OR_EQUAL(103, result);
}

// Main test runner
int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_filter_init);
    RUN_TEST(test_filter_single_value);
    RUN_TEST(test_filter_median_odd_count);
    RUN_TEST(test_filter_median_even_count);
    RUN_TEST(test_filter_removes_outliers);
    RUN_TEST(test_filter_size_change);
    RUN_TEST(test_filter_invalid_size);
    RUN_TEST(test_filter_buffer_overflow);
    RUN_TEST(test_filter_age_rotation);
    RUN_TEST(test_filter_consistent_values);
    RUN_TEST(test_filter_ascending_sequence);
    RUN_TEST(test_filter_descending_sequence);
    RUN_TEST(test_filter_noise_reduction);
    RUN_TEST(test_filter_spike_rejection);

    return UNITY_END();
}
