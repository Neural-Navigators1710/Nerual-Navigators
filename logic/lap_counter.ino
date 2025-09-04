// ===================================================
// === TURN COUNT (Lap Counter) ===
// ===================================================
int turnCount = 0;  // Counts turns based on TOF gaps

// Confirmation counters
int leftGapConfirm = 0;
int rightGapConfirm = 0;

// Flags for gap activity
bool turning = false;
enum TurnType { NONE, LEFT_TURN, RIGHT_TURN };
TurnType currentTurn = NONE;

unsigned long gapCooldownEnd = 0;  // cooldown between turns

// ===================================================
// === GAP Detection & Turn Counting ===
// ===================================================
void handleTOFLapCounter(uint16_t leftDist, uint16_t rightDist) {
  // --- GAP detection (only if not already turning and cooldown finished) ---
  if (!turning && millis() > gapCooldownEnd) {
    
    // Left Gap
    if (leftDist > 850) leftGapConfirm++;
    else leftGapConfirm = 0;

    if (leftGapConfirm > 0) {
      currentTurn = LEFT_TURN;
      turning = true;
      turnCount++;
      leftGapConfirm = 0;
      gapCooldownEnd = millis() + 1000; // prevent double-count
      Serial.print("🔢 Turn Count: ");
      Serial.println(turnCount);
    }

    // Right Gap
    if (rightDist > 850) rightGapConfirm++;
    else rightGapConfirm = 0;

    if (rightGapConfirm > 0) {
      currentTurn = RIGHT_TURN;
      turning = true;
      turnCount++;
      rightGapConfirm = 0;
      gapCooldownEnd = millis() + 1000; // prevent double-count
      Serial.print("🔢 Turn Count: ");
      Serial.println(turnCount);
    }
  }
}
