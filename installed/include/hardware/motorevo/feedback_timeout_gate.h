#pragma once
#include <cstdint>
namespace motorevo {
constexpr uint8_t kFeedbackMaxConsecutiveSuppress = 3;  // A4 fail-open bound
struct FeedbackTimeoutResult { bool timed_out; uint8_t streak; uint8_t suppress_run; };
// over_threshold: caller already applied the (now_ms >= last ? age : 0) guard,
//   so now_ms <= last  =>  over_threshold == false.
// confirm_frames == 0 is treated as 1 (debounce never disabled).
// suppress == true this cycle: health check ran late.
//   While suppress_run < kFeedbackMaxConsecutiveSuppress: hold -> {false, prev_streak,
//   suppress_run+1} (streak not advanced, not classified; matches PLAN A4).
//   Once the cap is reached the gate FAILS OPEN: it evaluates normally even while
//   suppress stays true, and pins suppress_run at the cap so it keeps evaluating
//   until a NON-suppressed cycle resets suppress_run to 0. A chronically-late
//   1 kHz safety check thus loses at most kFeedbackMaxConsecutiveSuppress (3)
//   cycles before FEEDBACK_TIMEOUT detection resumes — it can never be permanently
//   blinded (SAFETY condition load-not-chronic-suppress).
inline FeedbackTimeoutResult evaluateFeedbackTimeout(
    bool over_threshold, uint8_t prev_streak, uint8_t prev_suppress_run,
    uint8_t confirm_frames, bool suppress) {
    const uint8_t need = confirm_frames ? confirm_frames : static_cast<uint8_t>(1);
    if (suppress && prev_suppress_run < kFeedbackMaxConsecutiveSuppress) {
        return { false, prev_streak, static_cast<uint8_t>(prev_suppress_run + 1) };
    }
    const uint8_t sr = suppress ? kFeedbackMaxConsecutiveSuppress : static_cast<uint8_t>(0);
    if (!over_threshold) return { false, 0, sr };
    uint8_t s = (prev_streak < 255) ? static_cast<uint8_t>(prev_streak + 1) : prev_streak;
    return { s >= need, s, sr };
}
}  // namespace motorevo
