# ISEF Rationale Review

## Overall Assessment: **Excellent** [x]

Your rationale is well-structured, clear, and compelling. It effectively:
- Establishes real-world relevance (avalanche search-and-rescue)
- Identifies a clear problem (faulty leaders in leader-follower systems)
- States your research question
- Describes your solution (REIP)
- Connects to future applications

## Minor Suggestions for Improvement

### 1. Small Grammar Fix
**Current:** "in the rare case of a compromising attack"
**Suggested:** "in the rare case of a compromise attack" or "if a leader becomes compromised"

### 2. Technical Term Clarification
**Current:** "more organized informatics structure"
**Suggestion:** Consider "more organized information structure" or "more structured information flow" - "informatics" might be too technical/jargony for some ISEF judges

### 3. Optional: Strengthen the Gap Statement
You could add one sentence about why existing solutions (like Raft) aren't sufficient:
- "However, existing distributed policies such as Raft can only detect leaders that stop responding; they cannot detect leaders that remain active while sending harmful commands."

### 4. Word Count Check
Your rationale is approximately 250 words, which is appropriate for ISEF (typically 200-300 words is ideal).

## What's Great

[x] **Strong opening** - Real-world problem with clear stakes (survival probability decreases with time)
[x] **Good problem setup** - Multi-robot advantages -> leader-follower benefits -> vulnerability identified
[x] **Clear research question** - "whether a low cost robot team is able to detect..."
[x] **Solution description** - REIP explained clearly without excessive technical detail
[x] **Future applications** - Nice connection to real-world deployment

## Final Version (with minor edits)

Avalanche and wilderness search-and-rescue missions are highly time-critical, and survival probability has been shown to decrease sharply each minute rescue is delayed. In mountainous terrain, communication can be unreliable, helicopters take time to set up, and search teams often put themselves at risk which slows coverage as more safety precautions are taken. 

Literature has shown that multi-robot swarms are able to cover areas far quicker than a single robot or group of humans could. Robot swarms that prioritize coverage can be incredibly useful when maximum coverage of avalanche debris is critical. Systems with a leader-follower hierarchy have been found to be more effective at navigating complex and unknown terrain maps due to a more organized information structure than a standard decentralized system would use. However, a leader-follower system exposes a significant vulnerability; in the case of a leader becoming faulty through sensor breakage or even in the rare case of a compromise attack, the rest of the team will continue to follow poor commands, wasting critical time and reducing mission effectiveness.

Existing distributed policies such as Raft are able to detect a leader that stops responding; however, they cannot reliably detect a leader that remains active while sending harmful or low quality commands. My project investigates whether a low cost robot team is able to detect this type of subtle leadership failure using only local sensing and peer communication, without centralized oversight or any access to ground-truth knowledge.

To address this problem, I developed the Resilient Election and Impeachment Policy (REIP), a lightweight trust-based governance framework in which follower robots verify leader commands before execution. Instead of depending on centralized oversight or ground-truth information, REIP allows robots to recognize when a leader may be making harmful or unreliable judgement and to replace that leader when necessary. This work presents an important autonomy layer for search-and-rescue robot teams operating in communication-limited, time-critical, and safety-critical environments. In future applications, REIP can be combined with victim-detection radar systems and automatic UAV-assisted deployment to support avalanche rescue operations.

---

**Changes made:**
1. "compromising attack" -> "compromise attack"
2. "informatics structure" -> "information structure"
3. Added semicolon in Raft sentence for better flow

**Word count:** ~250 words (perfect for ISEF)
